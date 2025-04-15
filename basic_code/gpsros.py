import rospy
from flask import Flask, request
from sensor_msgs.msg import NavSatFix

app = Flask(__name__)
rospy.init_node('gps_web_receiver', anonymous=True)
gps_pub = rospy.Publisher('/gps_data', NavSatFix, queue_size=10)

@app.route('/', methods=['GET'])
def receive_gps():
    try:
        latitude = float(request.args.get('latitude'))
        longitude = float(request.args.get('longitude'))
        rospy.loginfo(f"📡 GPS 수신: 위도 {latitude}, 경도 {longitude}")

        msg = NavSatFix()
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = 0.0
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "gps"

        gps_pub.publish(msg)
        return "✅ GPS 데이터 수신 완료!"
    except Exception as e:
        return f"⚠️ 오류 발생: {e}"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)