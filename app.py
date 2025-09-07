# app.py
from flask import Flask, render_template, Response, request, jsonify
import cv2
import time
import RPi.GPIO as GPIO
import board
import busio
from i2c_lcd import I2cLcd
import socket
import requests
import threading

app = Flask(__name__)

# ????? ???????
SERVO_X_PIN = 17
SERVO_Y_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_X_PIN, GPIO.OUT)
GPIO.setup(SERVO_Y_PIN, GPIO.OUT)
servo_x = GPIO.PWM(SERVO_X_PIN, 50)  # 50Hz
servo_y = GPIO.PWM(SERVO_Y_PIN, 50)
servo_x.start(7.5)
servo_y.start(7.5)
#laser
LASER_PIN = 27  # ??? ??? ???? ??????? ???? ??????
GPIO.setup(LASER_PIN, GPIO.OUT)
GPIO.output(LASER_PIN, GPIO.LOW)
# ????? ????????
from picamera2 import Picamera2
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

# ???? ??????
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# ????? ???? LCD
I2C_ADDR = 0x27
I2C_NUM_ROWS = 2
I2C_NUM_COLS = 16
i2c = busio.I2C(board.SCL, board.SDA)
lcd = I2cLcd(i2c, I2C_ADDR, I2C_NUM_ROWS, I2C_NUM_COLS)
lcd.clear()
lcd.backlight_on()



last_angle_x = 90
last_angle_y = 90
face_coords = None
servo_x.ChangeDutyCycle(0)
servo_y.ChangeDutyCycle(0)

last_lcd_update = 0
lcd_update_interval = 1  # ?????

#modes
auto_mode = False

#BUZZER
buzzer_pin = 23  
GPIO.setup(buzzer_pin, GPIO.OUT)
buzzer_duration = 0.2  
buzzer_active = False
buzzer_start_time = 0
face_detected_prev = False

#ip of esp
esp_ip = "192.168.1.36"


#get location data
def check_internet():
    try:
        socket.create_connection(("8.8.8.8", 53), timeout=1)
        return True
    except:
        return False


line2 = "No internet"
try:
    if check_internet():
        res = requests.get("https://ipinfo.io", timeout=1)
        data = res.json()
        
        location = data['loc']  # ?????: "32.2211,35.0089"
        latitude, longitude = location.split(',')
        line2 = f"{float(latitude):.1f},{float(longitude):.1f}"

    else:
        lcd.putstr(line2.ljust(16))
    
except requests.RequestException:
    print("RequestException")
    

# --- ADDITIONS FOR ROTATING TOP SERVO ---
TOP_SERVO_PIN = 22
GPIO.setup(TOP_SERVO_PIN, GPIO.OUT)
top_servo = GPIO.PWM(TOP_SERVO_PIN, 50)  # 50Hz
top_servo.start(7.5)  # Move to middle (90 degrees)

def angle_to_duty(angle):
    return max(2.5, min(12.5, 2.5 + (angle / 180.0) * 10))

# Start at 90 degrees
top_servo_angle = 90
def set_top_servo(angle):
    global top_servo_angle
    angle = max(0, min(180, angle))
    duty = angle_to_duty(angle)
    top_servo.ChangeDutyCycle(duty)
    time.sleep(0.3)
    top_servo.ChangeDutyCycle(0)
    top_servo_angle = angle

# Move to center on boot
set_top_servo(90)


def compute_angles(x, y, frame_width=640, frame_height=480, invert=False):
    fx = frame_width // 2
    fy = frame_height // 2

    offset_x = x - fx
    offset_y = y - fy

#     if invert:
#         angle_x = int((-offset_x / fx) * 40 + 20)
#         angle_y = int((-offset_y / fy) * 45 + 65)
#     else:
#         angle_x = int((offset_x / fx) * 40 + 20)
#         angle_y = int((offset_y / fy) * 45 + 65)

    angle_x = int((-offset_x / fx) * 40 + 80)
    angle_y = int((-offset_y / fy) * 45 + 55)
    print(angle_x)
    print(angle_y)
    
    angle_x = max(0, min(180, angle_x))
    angle_y = max(0, min(180, angle_y))

    return angle_x, angle_y

def auto_mode_loop():
    global auto_mode, last_angle_x, last_angle_y
    while True:
        if auto_mode and face_coords:
            x, y, w, h = face_coords
            cx = x + w // 2
            cy = y + h // 2

            fx = 640 // 2
            fy = 480 // 2

            offset_x = cx - fx
            offset_y = cy - fy

            angle_x, angle_y = compute_angles(cx, cy, invert=True)
            
#             angle_x = int((-offset_x / fx) * 45 + 90)
#             angle_y = int((-offset_y / fy) * 45 + 90)
# 
#             angle_x = max(0, min(180, angle_x))
#             angle_y = max(0, min(180, angle_y))

            duty_x = angle_to_duty(angle_x)
            duty_y = angle_to_duty(angle_y)

            print(f"[AUTO] Moving to angle_x={angle_x}, angle_y={angle_y}")
            servo_x.ChangeDutyCycle(duty_x)
            servo_y.ChangeDutyCycle(duty_y)
            GPIO.output(LASER_PIN, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(LASER_PIN, GPIO.LOW)
            servo_x.ChangeDutyCycle(0)
            servo_y.ChangeDutyCycle(0)

            last_angle_x = angle_x
            last_angle_y = angle_y
            
            
        time.sleep(0.1)
        
        


def check_internet():
    try:
        socket.create_connection(("8.8.8.8", 53), timeout=1)
        return True
    except:
        return False

def get_fake_lat_lon(angle_x, angle_y):
    # ????? ???????? ??????? ????? ????? ?????? ???? ??? ??????? (????? ???)
    # Latitude range: -90 to 90
    # Longitude range: -180 to 180
    lat = (angle_y - 90) * 2  # ?????
    lon = (angle_x - 90) * 4  # ?????
    # ??? ????? ???? ??????
    lat = max(-90, min(90, lat))
    lon = max(-180, min(180, lon))
    return lat, lon

def update_lcd_display(face_detected, angle_x=None, angle_y=None):
    global last_lcd_update
    current_time = time.time()
    if current_time - last_lcd_update < lcd_update_interval:
        return
    last_lcd_update = current_time

    lcd.clear()

    if face_detected:
        
        lcd.set_cursor(0, 0)
        lcd.putstr("Target detected")
        lcd.set_cursor(0, 1)
        lcd.putstr(line2.ljust(16))
        print(f"[LCD] {line2}")  # Debug ?? ???????
        
    else:
        # ???? ???? ?????? ?? ?????? ?????
        pass




def gen_frames():
#     global last_angle_x, last_angle_y, face_coords
    global last_angle_x, last_angle_y, face_coords, face_detected_prev, buzzer_active, buzzer_start_time

    while True:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
        
        #buzzer turnon
        if faces is not None and len(faces) > 0:
            if not face_detected_prev:
                print("[ESP] Sending face_detected just once every update ")
                try:
                    requests.get(f"http://{esp_ip}/face_detected", timeout=0.3)
                except:
                    pass
                print(" turnon buzzer")
                GPIO.output(buzzer_pin, GPIO.HIGH)
                buzzer_active = True
                buzzer_start_time = time.time()
                face_detected_prev = True
        else:
            if face_detected_prev:
                print("[ESP] Sending face_lost")
                try:
                    requests.get(f"http://{esp_ip}/face_lost", timeout=0.3)
                except:
                    pass
                face_detected_prev = False

        # turn off
        if buzzer_active and (time.time() - buzzer_start_time) >= buzzer_duration:
            GPIO.output(buzzer_pin, GPIO.LOW)
            buzzer_active = False



        if len(faces) > 0:
            x, y, w, h = faces[0]
            face_coords = (x, y, w, h)
            cx = x + w // 2
            cy = y + h // 2

            fx = frame.shape[1] // 2
            fy = frame.shape[0] // 2

            offset_x = cx - fx
            offset_y = cy - fy
            
            angle_x, angle_y = compute_angles(cx, cy)

#             angle_x = int((offset_x / fx) * 45 + 90)
#             angle_y = int((offset_y / fy) * 45 + 90)
# 
#             angle_x = max(0, min(180, angle_x))
#             angle_y = max(0, min(180, angle_y))

            last_angle_x = angle_x
            last_angle_y = angle_y

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # ????? ???? LCD
            update_lcd_display(True, angle_x, angle_y)

        else:
            face_coords = None
            # ?? ???? ??????? ???? ??? ??

        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/set_mode', methods=['POST'])
def set_mode():
    global auto_mode
    data = request.get_json()
    auto_mode = (data.get("mode") == "auto")
    return jsonify({'mode': 'auto' if auto_mode else 'default'})

@app.route('/stop_all', methods=['POST'])
def stop_all():
    servo_x.ChangeDutyCycle(0)
    servo_y.ChangeDutyCycle(0)
    GPIO.output(LASER_PIN, GPIO.LOW)
    return '', 204

@app.route('/video_feed')
def video():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/face_status')
def face_status():
    if face_coords:
        x, y, w, h = face_coords
        cx = x + w // 2
        cy = y + h // 2
        return jsonify({
            'face_detected': True,
            'coords': [int(x), int(y), int(w), int(h)],
            'center': [int(cx), int(cy)],
            'gps': line2
        })
    else:
        return jsonify({'face_detected': False})


@app.route('/face_image')
def face_image():
    if face_coords:
        frame = picam2.capture_array()
        x, y, w, h = face_coords
        face_img = frame[y:y+h, x:x+w]
        ret, buffer = cv2.imencode('.jpg', face_img)
        return Response(buffer.tobytes(), mimetype='image/jpeg')
    return '', 204


@app.route('/aim_laser', methods=['POST'])
def aim_lasers():
    
    
    #laser
    GPIO.output(LASER_PIN, GPIO.HIGH)
    time.sleep(0.3)
    GPIO.output(LASER_PIN, GPIO.LOW)
    return jsonify({'message': 'Laser fired'})




@app.route('/move_servo', methods=['POST'])
def move_servo():
    global last_angle_x, last_angle_y
    try:
        data = request.get_json()

        if 'x' not in data or 'y' not in data:
            return jsonify({'error': 'Missing x or y'}), 400

        x = data['x']
        y = data['y']

        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
            return jsonify({'error': 'Invalid x or y values'}), 400

        fx = 640 // 2
        fy = 480 // 2

        offset_x = x - fx
        offset_y = y - fy

#         angle_x = int((-offset_x / fx) * 45 + 90)
#         angle_y = int((-offset_y / fy) * 45 + 90)
# 
#         angle_x = max(0, min(180, angle_x))
#         angle_y = max(0, min(180, angle_y))

        angle_x, angle_y = compute_angles(x, y, invert=True)


        duty_x = angle_to_duty(angle_x)
        duty_y = angle_to_duty(angle_y)

        print(f"[DEBUG] x={x}, y={y}, angle_x={angle_x}, angle_y={angle_y}, duty_x={duty_x}, duty_y={duty_y}")

        servo_x.ChangeDutyCycle(duty_x)
        servo_y.ChangeDutyCycle(duty_y)
        time.sleep(0.5)
        servo_x.ChangeDutyCycle(0)
        servo_y.ChangeDutyCycle(0)

        last_angle_x = angle_x
        last_angle_y = angle_y

        return jsonify({'message': f'Moving to x={x}, y={y}, yaw={angle_x}, pitch={angle_y}'})

    except Exception as e:
        print(f"[ERROR] {str(e)}")
        return jsonify({'error': str(e)}), 500

@app.route('/rotate_top_servo', methods=['POST'])
def rotate_top_servo():
    global top_servo_angle
    try:
        data = request.get_json()
        direction = data.get("direction")  
        if direction == "left":
            top_servo_angle = max(0, top_servo_angle - 10)
        elif direction == "right":
            top_servo_angle = min(180, top_servo_angle + 10)
        else:
            return jsonify({"error": "Invalid direction"}), 400

        set_top_servo(top_servo_angle)
        return jsonify({"angle": top_servo_angle})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


if __name__ == '__main__':
    try:
        threading.Thread(target=auto_mode_loop, daemon=True).start()
        app.run(host='0.0.0.0', port=5000)
    finally:
        servo_x.stop()
        servo_y.stop()
        GPIO.cleanup()