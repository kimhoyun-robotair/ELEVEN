import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {RoomEnvironment} from 'three/addons/environments/RoomEnvironment.js';
import {ElevatorController} from './elevator.mjs';

const $=id=>document.getElementById(id), viewport=$('viewport');
const worlds={office:{name:'Atrium Office',tag:'01 / WORKPLACE',description:'4개 층 · 사무실 · 회의실 · 공용 공간',heights:[0,3.6,7.2,10.8],positions:[[-10.5,8.5],[-3.5,8.5],[3.5,8.5],[10.5,8.5]],types:['opaque','glass','opaque','glass']},house:{name:'Garden Residence',tag:'02 / RESIDENCE',description:'3개 층 · 주방 · 거실 · 침실 · 실내 엘리베이터',heights:[0,3.4,6.8],positions:[[-4,7.4],[4,7.4]],types:['opaque','glass']}};
let worldKey='office',requestedWorldKey='office',world,model,rigs=[],active=0,floor=0,mode='lobby',loadSerial=0,lastTime=0,toastTimer;
let objects=[],floorGroups=[],ceilings=[],clickables=[],displays=[],pointerStart=null,keys=new Set(),drag=null;
let renderer;
try{renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'high-performance'});}catch(error){$('load-detail').textContent='WebGL 2를 지원하는 브라우저에서 열어 주세요. '+error.message;throw error;}
renderer.setPixelRatio(Math.min(devicePixelRatio,1.6));renderer.outputColorSpace=THREE.SRGBColorSpace;
renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.05;
renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;
viewport.prepend(renderer.domElement);
const scene=new THREE.Scene();scene.background=new THREE.Color('#b6c5cc');
const camera=new THREE.PerspectiveCamera(63,1,.04,250);camera.up.set(0,0,1);
const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.dampingFactor=.075;controls.maxDistance=100;controls.minDistance=.15;controls.maxPolarAngle=Math.PI*.94;
const pmrem=new THREE.PMREMGenerator(renderer), roomEnv=new RoomEnvironment();scene.environment=pmrem.fromScene(roomEnv,.04).texture;roomEnv.dispose();pmrem.dispose();scene.environmentIntensity=.75;
const skyFill=new THREE.HemisphereLight(0xe7f4ff,0x7b766a,1.25);skyFill.position.set(0,0,1);scene.add(skyFill);scene.environmentRotation.x=Math.PI/2;
const sun=new THREE.DirectionalLight(0xfff2d8,3);sun.position.set(-15,-18,28);sun.castShadow=true;sun.shadow.mapSize.set(2048,2048);Object.assign(sun.shadow.camera,{left:-22,right:22,top:22,bottom:-22,near:.1,far:80});sun.shadow.normalBias=.025;scene.add(sun);
const ray=new THREE.Raycaster(),mouse=new THREE.Vector2(),q=new THREE.Quaternion(),v=new THREE.Vector3();
const states={idle:'대기',closing:'문 닫는 중',moving:'이동 중',opening:'문 여는 중',dwell:'문 열림'};
function notify(message){$('toast').textContent=message;$('toast').classList.add('visible');clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').classList.remove('visible'),2800);}
function prop(o,name){return o.userData?.['testbed_'+name]??o.userData?.['testbed:'+name];}
function semantic(o){return prop(o,'path')||'';}
function move(o,x=0,y=0,z=0){if(!o)return;o.position.copy(o.userData.restPosition);v.set(x,y,z);o.parent.getWorldQuaternion(q).invert();v.applyQuaternion(q);o.position.add(v);}
function bindModel(){
 objects=[];floorGroups=[];ceilings=[];clickables=[];displays=[];
 model.traverse(o=>{objects.push(o);o.userData.restPosition=o.position.clone();
  const p=semantic(o);const fm=p.match(/^\/World\/(?:Building|Furniture)\/Floor_(\d+)$/)||p.match(/^\/World\/Elevators\/E\d+\/(?:LandingDoors|HallButtons)\/Floor_(\d+)$/);
  if(prop(o,'floorGroup')!==undefined||fm)floorGroups.push({object:o,floor:Number(prop(o,'floorGroup')??fm[1])});
  if(prop(o,'ceiling')||o.name==='Building_Roof')ceilings.push(o);
  if(prop(o,'displayFloor')!==undefined)displays.push(o);
  if(o.isMesh){o.castShadow=true;o.receiveShadow=true;if(prop(o,'button')){o.material=o.material.clone();o.material.emissive?.setRGB(0,0,0);clickables.push(o);}if(o.material?.transmission>0){o.material.thickness=.035;o.material.envMapIntensity=1.25;}}
  if(o.isLight)o.intensity=Math.min(o.intensity,40);
 });
 const find=p=>objects.find(o=>semantic(o)===p);
 rigs=world.positions.map((xy,i)=>{
  const id='E'+(i+1),path='/World/Elevators/'+id;
  const r={id,xy,type:world.types[i],controller:new ElevatorController(world.heights),cabin:find(path+'/Cabin'),doors:[find(path+'/Cabin/Doors/Left'),find(path+'/Cabin/Doors/Right')],landings:world.heights.map((_,f)=>[find(path+`/LandingDoors/Floor_${f}/Left`),find(path+`/LandingDoors/Floor_${f}/Right`)])};
  if(!r.cabin||r.doors.some(d=>!d)||r.landings.some(pair=>pair.some(d=>!d)))throw new Error(`${id}: 엘리베이터 모델 연결이 불완전합니다.`);
  return r;
 });
 if(!clickables.length)throw new Error('3D 버튼 메타데이터가 누락되었습니다.');
}
// Batch repeated static furniture meshes by geometry/material within each floor.
// Their original hierarchy remains available for metadata, with exact transforms.
function batchFurniture(){
 for(const {object:group} of floorGroups){
  if(!semantic(group).includes('/Furniture/')&&!group.name.startsWith('Furniture_Floor_'))continue;
  const batches=new Map();group.updateWorldMatrix(true,true);
  group.traverse(o=>{if(!o.isMesh||!o.visible||Array.isArray(o.material)||prop(o,'button'))return;const k=o.geometry.uuid+'_'+o.material.uuid;if(!batches.has(k))batches.set(k,[]);batches.get(k).push(o);});
  const inverse=new THREE.Matrix4().copy(group.matrixWorld).invert();
  for(const meshes of batches.values()){
   if(meshes.length<3)continue;
   const batch=new THREE.InstancedMesh(meshes[0].geometry,meshes[0].material,meshes.length);batch.name='StaticFurnitureBatch';batch.castShadow=true;batch.receiveShadow=true;
   meshes.forEach((m,i)=>{batch.setMatrixAt(i,new THREE.Matrix4().multiplyMatrices(inverse,m.matrixWorld));m.visible=false;});batch.instanceMatrix.needsUpdate=true;batch.computeBoundingSphere();group.add(batch);
  }
 }
 model.traverse(o=>{o.updateMatrix();o.matrixAutoUpdate=false;});
 for(const r of rigs){r.cabin.matrixAutoUpdate=true;r.doors.forEach(o=>o.matrixAutoUpdate=true);r.landings.flat().forEach(o=>o.matrixAutoUpdate=true);}
}
async function loadWorld(key){
 requestedWorldKey=key;const serial=++loadSerial;document.querySelector('aside').inert=true;$('loading').hidden=false;$('retry').hidden=true;$('load-detail').textContent='Blender 모델과 재질 준비';
 try{
  const gltf=await new GLTFLoader().loadAsync(`./assets/${key}.gltf`,p=>{if(serial===loadSerial&&p.total)$('load-detail').textContent=`${Math.round(100*p.loaded/p.total)}% · ${(p.total/1048576).toFixed(1)} MB`;});
  if(serial!==loadSerial){dispose(gltf.scene);return;}
  worldKey=key;world=worlds[key];
  if(model){scene.remove(model);dispose(model);}
  model=new THREE.Group();model.rotation.x=Math.PI/2;model.add(gltf.scene);scene.add(model);model.updateMatrixWorld(true);bindModel();batchFurniture();floor=0;active=0;
  $('world-title').textContent=world.name;$('world-tag').textContent=world.tag;$('world-description').textContent=world.description;
  document.querySelectorAll('[data-world]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.world===key)));
  $('floor').replaceChildren(...world.heights.map((_,i)=>new Option(`${i+1}층${i===0?' · Ground floor':''}`,i)));
  $('elevator').replaceChildren(...rigs.map((r,i)=>new Option(`${r.id} · ${r.type==='glass'?'유리 파노라마':'불투명 스테인리스'}`,i)));
  $('floor-buttons').replaceChildren(...world.heights.map((_,i)=>{const b=document.createElement('button');b.textContent=i+1;b.setAttribute('aria-label',`${i+1}층 선택`);b.setAttribute('aria-pressed','false');b.onclick=()=>press('floor',i);return b;}));
  $('loading').hidden=true;document.querySelector('aside').inert=false;$('cutaway').checked=false;$('obstruction').checked=false;setMode('lobby');updateUi(rigs[0].controller.snapshot());
 }catch(error){if(serial!==loadSerial)return;console.error(error);$('loading').hidden=false;$('load-detail').textContent='환경을 불러오지 못했습니다. '+error.message;$('retry').hidden=false;document.querySelector('aside').inert=false;}
}
function dispose(root){const geometries=new Set(),materials=new Set(),textures=new Set();root.traverse(o=>{if(o.geometry)geometries.add(o.geometry);for(const m of (Array.isArray(o.material)?o.material:[o.material]))if(m){materials.add(m);Object.values(m).forEach(v=>{if(v?.isTexture)textures.add(v);});}});geometries.forEach(g=>g.dispose());materials.forEach(m=>m.dispose());textures.forEach(t=>{t.source?.data?.close?.();t.dispose();});}
function setMode(next){mode=next;controls.enabled=mode!=='walk';keys.clear();$('walk-pad').hidden=mode!=='walk';$('ride').classList.toggle('active',mode==='ride');$('ride').innerHTML=mode==='ride'?'탑승 중 · 로비로 나가기 <span>↗</span>':'탑승하여 내부 보기 <span>↗</span>';
 ['lobby','overview','walk'].forEach(id=>$(id).classList.toggle('selected',mode===id));$('mode-label').textContent=mode.toUpperCase();
 $('navigation-hint').textContent=mode==='walk'?'WASD / 화살표: 이동 · 드래그: 시선 · Q/E: 높이 이동':mode==='ride'?'엘리베이터 내부 · 드래그: 둘러보기 · 실제 버튼 클릭':'드래그: 둘러보기 · 스크롤: 확대 · 3D 버튼 클릭: 호출';
 if(!rigs.length)return;const r=rigs[active],z=world.heights[floor];
 if(mode==='overview'){camera.position.set(33,-38,30);controls.target.set(0,0,5);$('cutaway').checked=true;}
 else if(mode==='ride'){const s=r.controller.snapshot();camera.position.set(r.xy[0]-.35,r.xy[1]-.6,s.position+1.6);controls.target.set(r.xy[0]+.55,r.xy[1]+.95,s.position+1.45);r.followZ=s.position;}
 else{camera.position.set(r.xy[0]+.35,r.xy[1]-3.8,z+1.65);controls.target.set(r.xy[0],r.xy[1]-1.4,z+1.5);}
 camera.lookAt(controls.target);controls.update();applyVisibility();updateLabels();
}
function applyVisibility(){const cut=$('cutaway').checked;const current=mode==='ride'?rigs[active]?.controller.snapshot().floor??floor:floor;floorGroups.forEach(g=>g.object.visible=!cut||g.floor<=current);ceilings.forEach(o=>o.visible=!cut);}
function updateLabels(){const f=mode==='ride'?rigs[active]?.controller.snapshot().floor??floor:floor;$('view-label').textContent=`${worldKey.toUpperCase()} / FLOOR ${String(f+1).padStart(2,'0')}`;$('lift-type').textContent=rigs[active]?.type==='glass'?'유리':'불투명';}
function press(type,f=floor,direction,idx=active){const r=rigs[idx];if(!r)return;let accepted=true;
 if(type==='floor')r.controller.requestFloor(Number(f));else if(type==='hall')r.controller.requestHall(Number(f),direction);else if(type==='open')accepted=r.controller.openDoors();else if(type==='close')accepted=r.controller.closeDoors();else if(type==='alarm'){r.controller.pressAlarm();beep();}
 if(!accepted&&type==='open')notify('이동 중에는 문을 열 수 없습니다.');else if(type==='floor'||type==='hall')notify(`${r.id} · ${Number(f)+1}층 요청`);else if(type==='alarm')notify('경보 버튼이 눌렸습니다.');
}
let audioContext;function beep(){try{audioContext??=new(window.AudioContext||window.webkitAudioContext)();audioContext.resume();const osc=audioContext.createOscillator(),gain=audioContext.createGain();osc.frequency.value=660;gain.gain.value=.04;osc.connect(gain).connect(audioContext.destination);osc.start();gain.gain.exponentialRampToValueAtTime(.0001,audioContext.currentTime+.5);osc.stop(audioContext.currentTime+.5);}catch{}}
function illuminate(button,s){const type=prop(button,'button'),f=Number(prop(button,'floor')??0),d=prop(button,'direction');let lit=type==='floor'?s.cabinLights[f]:type==='hall'?s.hallLights[f]?.[d]:type==='open'?s.openButtonLit:type==='close'?s.closeButtonLit:s.alarmLit;if(button.userData.lastLit===lit)return;button.userData.lastLit=lit;button.material.emissive?.setRGB(...(lit?[3,1.65,.32]:[0,0,0]));}
function updateUi(s){$('lift-floor').textContent=String(s.floor+1).padStart(2,'0');$('lift-state').textContent=s.obstruction?'출입구 장애물':states[s.state];$('lift-velocity').textContent=`${Math.abs(s.velocity).toFixed(2)} m/s`;$('lift-direction').textContent=s.velocity>0?'↑':s.velocity<0?'↓':'—';[...$('floor-buttons').children].forEach((b,i)=>{b.classList.toggle('lit',s.cabinLights[i]);b.setAttribute('aria-pressed',String(s.cabinLights[i]));});['up','down'].forEach(d=>{const b=$('hall-'+d);b.disabled=(d==='up'&&floor===world.heights.length-1)||(d==='down'&&floor===0);b.classList.toggle('lit',!!s.hallLights[floor]?.[d]);});['open','close','alarm'].forEach(id=>$(id).classList.toggle('lit',!!s[id==='alarm'?'alarmLit':id+'ButtonLit']));}
function animate(now){requestAnimationFrame(animate);const dt=Math.min((now-lastTime)/1000||0,.1);lastTime=now;
 if(model&&rigs.length){rigs.forEach((r,idx)=>{r.controller.tick(dt);const s=r.controller.snapshot();move(r.cabin,0,0,s.position);r.doors.forEach((d,i)=>move(d,(i===0?-1:1)*.72*s.doorOpen));r.landings.forEach((pair,f)=>pair.forEach((d,i)=>move(d,(i===0?-1:1)*.72*(s.floor===f?s.doorOpen:0))));clickables.filter(o=>prop(o,'elevatorId')===r.id).forEach(o=>illuminate(o,s));displays.filter(o=>prop(o,'elevatorId')===r.id).forEach(o=>o.visible=Number(prop(o,'displayFloor'))===s.floor);if(idx===active){if(mode==='ride'){const dz=s.position-(r.followZ??s.position);camera.position.z+=dz;controls.target.z+=dz;r.followZ=s.position;applyVisibility();updateLabels();}updateUi(s);}});
  if(mode==='walk'){const forward=new THREE.Vector3();camera.getWorldDirection(forward);forward.z=0;forward.normalize();const right=new THREE.Vector3().crossVectors(forward,camera.up).normalize();const speed=(keys.has('Shift')?4:2)*dt;const delta=new THREE.Vector3();if(keys.has('w')||keys.has('ArrowUp')||keys.has('forward'))delta.addScaledVector(forward,speed);if(keys.has('s')||keys.has('ArrowDown')||keys.has('backward'))delta.addScaledVector(forward,-speed);if(keys.has('a')||keys.has('ArrowLeft')||keys.has('left'))delta.addScaledVector(right,-speed);if(keys.has('d')||keys.has('ArrowRight')||keys.has('right'))delta.addScaledVector(right,speed);if(keys.has('e'))delta.z+=speed;if(keys.has('q'))delta.z-=speed;camera.position.add(delta);controls.target.copy(camera.position).add(forward);}
 }
 if(controls.enabled)controls.update();renderer.render(scene,camera);
}
new ResizeObserver(()=>{const w=viewport.clientWidth,h=viewport.clientHeight;renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();}).observe(viewport);
document.querySelectorAll('[data-world]').forEach(b=>b.onclick=()=>loadWorld(b.dataset.world));$('retry').onclick=()=>loadWorld(requestedWorldKey);
$('floor').onchange=()=>{floor=Number($('floor').value);setMode('lobby');};$('elevator').onchange=()=>{active=Number($('elevator').value);$('obstruction').checked=rigs[active].controller.snapshot().obstruction;setMode('lobby');};
['lobby','overview','walk'].forEach(id=>$(id).onclick=()=>setMode(id));$('ride').onclick=()=>setMode(mode==='ride'?'lobby':'ride');$('reset').onclick=()=>setMode(mode);$('cutaway').onchange=applyVisibility;
['open','close','alarm'].forEach(id=>$(id).onclick=()=>press(id));$('hall-up').onclick=()=>press('hall',floor,'up');$('hall-down').onclick=()=>press('hall',floor,'down');$('obstruction').onchange=()=>rigs[active]?.controller.setObstruction($('obstruction').checked);
renderer.domElement.addEventListener('pointerdown',e=>{pointerStart={x:e.clientX,y:e.clientY};if(mode==='walk'){drag={x:e.clientX,y:e.clientY};renderer.domElement.setPointerCapture(e.pointerId);}});
renderer.domElement.addEventListener('pointermove',e=>{if(!drag||mode!=='walk')return;const dx=(e.clientX-drag.x)*.004,dy=(e.clientY-drag.y)*.004;const dir=new THREE.Vector3();camera.getWorldDirection(dir);const yaw=Math.atan2(dir.y,dir.x)-dx,pitch=THREE.MathUtils.clamp(Math.asin(dir.z)-dy,-1.4,1.4);dir.set(Math.cos(yaw)*Math.cos(pitch),Math.sin(yaw)*Math.cos(pitch),Math.sin(pitch));camera.lookAt(camera.position.clone().add(dir));drag={x:e.clientX,y:e.clientY};});
renderer.domElement.addEventListener('pointerup',e=>{drag=null;if(!pointerStart||Math.hypot(e.clientX-pointerStart.x,e.clientY-pointerStart.y)>5)return;const rect=renderer.domElement.getBoundingClientRect();mouse.set((e.clientX-rect.left)/rect.width*2-1,-(e.clientY-rect.top)/rect.height*2+1);ray.setFromCamera(mouse,camera);const hits=ray.intersectObjects(model?[model]:[],true).filter(hit=>{let o=hit.object;while(o){if(!o.visible)return false;o=o.parent;}return true;});for(const hit of hits){let b=hit.object;let button=prop(b,'button')?b:null;for(let a=b.parent;!button&&a;a=a.parent){if(/\/(?:Panel\/(?:Floor_\d+|Open|Close|Alarm)|HallButtons\/Floor_\d+\/(?:Up|Down))$/.test(semantic(a)))button=clickables.find(o=>o.parent===a);}if(button){const idx=rigs.findIndex(r=>r.id===prop(button,'elevatorId'));if(idx>=0)press(prop(button,'button'),prop(button,'floor'),prop(button,'direction'),idx);break;}if(b.isMesh&&(b.material?.transmission??0)<.5&&(b.material?.opacity??1)>.5)break;}});
document.addEventListener('keydown',e=>{if(mode!=='walk'||/INPUT|SELECT|TEXTAREA/.test(e.target.tagName))return;if(['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','w','a','s','d','q','e','Shift'].includes(e.key)){e.preventDefault();keys.add(e.key);}});document.addEventListener('keyup',e=>keys.delete(e.key));window.addEventListener('blur',()=>{keys.clear();drag=null;});document.querySelectorAll('[data-move]').forEach(b=>{b.onpointerdown=e=>{e.preventDefault();keys.add(b.dataset.move);b.setPointerCapture(e.pointerId);};b.onpointerup=b.onpointercancel=()=>keys.delete(b.dataset.move);});
loadWorld('office');requestAnimationFrame(animate);
