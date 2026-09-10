// Run with: node --test tests/elevator.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import {ElevatorController} from '../simulation/elevator.mjs';

const makeCar = options => new ElevatorController([0.12, 3.92, 7.72, 11.52], options);
function advanceUntil(car, predicate) {
  for (let i = 0; i < 4500; i++) {
    const snapshot = car.tick(0.02);
    if (predicate(snapshot)) return snapshot;
  }
  assert.fail(`condition not reached: ${JSON.stringify(car.snapshot())}`);
}

test('only requested lamps light; duplicate calls remain unique and clear on full opening', () => {
  const car = makeCar();
  assert.deepEqual(car.snapshot().cabinLights, [false, false, false, false]);
  assert.equal(car.requestFloor(2), true);
  assert.equal(car.requestFloor(2), false);
  assert.equal(car.requestHall(1, 'down'), true);
  assert.equal(car.requestHall(1, 'down'), false);
  assert.deepEqual(car.snapshot().queue, [2, 1]);
  assert.deepEqual(car.snapshot().cabinLights, [false, false, true, false]);
  assert.deepEqual(car.snapshot().hallLights[1], {up: false, down: true});
  const landed = advanceUntil(car, s => s.floor === 2 && s.state === 'opening');
  assert.equal(landed.cabinLights[2], true);
  const opened = advanceUntil(car, s => s.floor === 2 && s.state === 'dwell');
  assert.equal(opened.cabinLights[2], false);
  assert.equal(opened.hallLights[1].down, true);
  car.tick(60);
  assert.equal(car.snapshot().floor, 1);
  assert.equal(car.snapshot().state, 'idle');
  assert.deepEqual(car.snapshot().queue, []);
  assert.equal(car.snapshot().hallLights.some(c => c.up || c.down), false);
});

test('same landing groups independent cabin and hall requests', () => {
  const car = makeCar();
  car.requestHall(1, 'up');
  car.requestHall(1, 'down');
  car.requestFloor(1);
  assert.deepEqual(car.snapshot().queue, [1]);
  assert.deepEqual(car.snapshot().hallLights[1], {up: true, down: true});
  car.tick(60);
  assert.equal(car.snapshot().cabinLights[1], false);
  assert.deepEqual(car.snapshot().hallLights[1], {up: false, down: false});
});

test('motion respects door interlocks, speed, acceleration, and landing bounds', () => {
  const car = makeCar({maxSpeed: 1.4, acceleration: 0.7});
  car.openDoors();
  car.tick(2);
  car.requestFloor(3);
  let previous = car.snapshot();
  let observedMotion = false;
  for (let i = 0; i < 4500; i++) {
    const current = car.tick(0.02);
    assert.ok(Math.abs(current.velocity) <= 1.4 + 1e-10);
    assert.ok(Math.abs(current.velocity - previous.velocity) <= 0.7 * 0.02 + 1e-9);
    assert.ok(current.position >= 0.12 - 1e-10 && current.position <= 11.52 + 1e-10);
    if (current.state === 'moving' || Math.abs(current.velocity) > 1e-10) {
      observedMotion = true;
      assert.equal(current.doorOpen, 0);
    }
    if (current.doorOpen > 0) {
      assert.ok(Math.abs(current.position - car.floorHeights[current.floor]) < 1e-10);
      assert.equal(current.velocity, 0);
    }
    previous = current;
  }
  assert.equal(observedMotion, true);
});

test('analytic trajectory is invariant to frame partition', () => {
  const a = makeCar({maxSpeed: 1.4, acceleration: 0.7});
  const b = makeCar({maxSpeed: 1.4, acceleration: 0.7});
  for (const car of [a, b]) { car.requestFloor(3); car.requestFloor(1); }
  a.tick(17.345);
  for (let i = 0; i < 3469; i++) b.tick(0.005);
  const sa = a.snapshot(), sb = b.snapshot();
  assert.equal(sa.state, sb.state);
  assert.deepEqual(sa.queue, sb.queue);
  for (const field of ['position', 'velocity', 'doorOpen']) assert.ok(Math.abs(sa[field] - sb[field]) < 1e-9);
});

test('obstruction reopens and holds at landing without losing requests', () => {
  const car = makeCar();
  car.openDoors();
  car.tick(1.8);
  car.requestFloor(2);
  car.closeDoors();
  car.tick(0.7);
  const before = car.snapshot().doorOpen;
  car.setObstruction(true);
  car.tick(0.3);
  assert.ok(car.snapshot().doorOpen > before);
  car.tick(100);
  assert.equal(car.snapshot().state, 'dwell');
  assert.equal(car.snapshot().floor, 0);
  assert.equal(car.snapshot().doorOpen, 1);
  assert.equal(car.snapshot().cabinLights[2], true);
  assert.equal(car.closeDoors(), false);
  car.setObstruction(false);
  car.tick(60);
  assert.equal(car.snapshot().floor, 2);
  assert.equal(car.snapshot().state, 'idle');
});

test('manual door commands and obstruction never open between floors', () => {
  const car = makeCar();
  car.requestFloor(3);
  car.tick(1);
  assert.equal(car.openDoors(), false);
  assert.equal(car.closeDoors(), false);
  car.setObstruction(true);
  car.tick(1);
  assert.equal(car.snapshot().state, 'moving');
  assert.equal(car.snapshot().doorOpen, 0);
  car.tick(60);
  assert.equal(car.snapshot().floor, 3);
  assert.equal(car.snapshot().doorOpen, 1);
});

test('current landing call reopens closing doors and clears at full opening', () => {
  const car = makeCar();
  car.requestFloor(0);
  assert.equal(car.snapshot().cabinLights[0], true);
  car.tick(1.8);
  assert.equal(car.snapshot().cabinLights[0], false);
  car.requestFloor(0);
  assert.equal(car.snapshot().cabinLights[0], false);
  car.closeDoors();
  car.tick(0.6);
  car.requestFloor(0);
  assert.equal(car.snapshot().state, 'opening');
  car.tick(1);
  assert.equal(car.snapshot().doorOpen, 1);
  assert.deepEqual(car.snapshot().queue, []);
});

test('control LEDs follow manual presses and expire independently', () => {
  const car = makeCar();
  car.requestFloor(2);
  car.tick(20);
  assert.equal(car.snapshot().openButtonLit, false);
  assert.equal(car.snapshot().closeButtonLit, false);
  car.openDoors();
  car.pressAlarm();
  assert.equal(car.snapshot().openButtonLit, true);
  assert.equal(car.snapshot().alarmLit, true);
  car.tick(0.81);
  assert.equal(car.snapshot().openButtonLit, false);
  assert.equal(car.snapshot().alarmLit, true);
  car.closeDoors();
  assert.equal(car.snapshot().closeButtonLit, true);
  car.tick(1);
  assert.equal(car.snapshot().closeButtonLit, false);
  assert.equal(car.snapshot().alarmLit, false);
});

test('snapshots cannot mutate controller arrays or hall direction records', () => {
  const car = makeCar();
  car.requestFloor(2);
  const snapshot = car.snapshot();
  snapshot.cabinLights[2] = false;
  snapshot.hallLights[0].up = true;
  snapshot.queue.length = 0;
  assert.deepEqual(car.snapshot().queue, [2]);
  assert.equal(car.snapshot().cabinLights[2], true);
  assert.equal(car.snapshot().hallLights[0].up, false);
});

test('invalid requests reject; short triangular trip and zero dwell finish safely', () => {
  for (const heights of [[0], [0, 0], [2, 1], [0, Infinity], [0, true]]) {
    assert.throws(() => new ElevatorController(heights));
  }
  const car = makeCar();
  for (const floor of [-1, 4, 0.5, true]) assert.throws(() => car.requestFloor(floor));
  for (const [floor, direction] of [[0, 'down'], [3, 'up'], [1, 'left']]) assert.throws(() => car.requestHall(floor, direction));
  for (const dt of [-1, NaN, Infinity, true]) assert.throws(() => car.tick(dt));
  const tiny = new ElevatorController([0, 0.01], {dwellSeconds: 0});
  tiny.requestFloor(1);
  tiny.tick(20);
  assert.equal(tiny.snapshot().position, 0.01);
  assert.equal(tiny.snapshot().state, 'idle');
});
