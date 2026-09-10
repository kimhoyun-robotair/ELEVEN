from pathlib import Path
import sys,time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from PIL import Image as PillowImage
out=Path(sys.argv[1]);out.mkdir(exist_ok=True,parents=True)
rclpy.init();node=rclpy.create_node('amr_capture_evidence');received=set()
def capture(side,msg):
    if side in received:return
    if msg.encoding!='rgb8':raise RuntimeError(msg.encoding)
    image=PillowImage.frombytes('RGB',(msg.width,msg.height),bytes(msg.data),'raw','RGB',msg.step)
    image.save(out/(side+'.png'));received.add(side);print(side,msg.width,msg.height,flush=True)
subs=[node.create_subscription(Image,f'/{s}_camera/color/image_raw',lambda msg,side=s:capture(side,msg),qos_profile_sensor_data) for s in ['front','rear']]
end=time.monotonic()+15
while len(received)<2 and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if len(received)==2 else 1)
