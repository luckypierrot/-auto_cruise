# -*- coding: utf-8 -*-
#!/usr/bin/python3
import cv2
import math
import numpy as np
import struct
import time
import pigpio
import threading
from multiprocessing import Process,Value



#	@brief		SIXAXISパラメータ取得用
EVENT_FORMAT  = 'LhBB'
EVENT_SIZE    = struct.calcsize(EVENT_FORMAT)

# @brief	モーター制御用GPIO定義
GPIO_MOTOR_PWM       = 23
GPIO_STEERING_PWM    = 24
GPIO_MOTOR_CTRL1     = 16
GPIO_MOTOR_CTRL2     = 20
GPIO_MOTOR_STBY      = 21

# @brief	モーター制御用GPIO定義
PWM_RANGE            = 100
PWM_FREQ             = 50
POWER_OFF            = 0
POWER_ON             = 1
STEERING_LEFT        = 1200
STEERING_FORWORD     = 1500
STEERING_RIGHT       = 1800

# @brief	進行方向定義
DIR_FORWARD          = 1
DIR_STOP             = 0
DIR_BACK             = -1

# @brief	車両の制御開始/停止
DRIVE_START          = 1
DRIVE_STOP           = 0

# @brief	カメラ設定値
CAM_SETTING_WIDTH    = 640
CAM_SETTING_HEIGHT   = 480
CAM_SETTING_FPS      = 24




#####################################################################
#	@class		auto_cruise
#	@brief		自動運転制御クラス
#####################################################################
class auto_cruise:

  #	@brief	privateパラメータ
  drive_steer          = 0
  drive_motor          = 0
  drive_active         = 0
  pi                   = 0


#####################################################################
#	@fn		    __init__
#	@param		無し
#	@brief		コンストラクタ
#####################################################################
  def __init__(self):
    print( "init start" )

    self.pi = pigpio.pi()

    # ポート入出力設定
    self.pi.set_mode( GPIO_MOTOR_PWM,    pigpio.OUTPUT)
    self.pi.set_mode( GPIO_STEERING_PWM, pigpio.OUTPUT)
    self.pi.set_mode( GPIO_MOTOR_CTRL1,  pigpio.OUTPUT)
    self.pi.set_mode( GPIO_MOTOR_CTRL2,  pigpio.OUTPUT)
    self.pi.set_mode( GPIO_MOTOR_STBY,   pigpio.OUTPUT)

    # 制御信号の初期化
    self.pi.write( GPIO_MOTOR_CTRL1, 0 )
    self.pi.write( GPIO_MOTOR_CTRL2, 0 )
    self.pi.write( GPIO_MOTOR_STBY,  0 )

    # PWMレンジ設定 0-100%
    self.pi.set_PWM_range( GPIO_MOTOR_PWM,    PWM_RANGE )
    self.pi.set_PWM_range( GPIO_STEERING_PWM, PWM_RANGE )

    # PWM周波数設定 50Hz
    self.pi.set_PWM_frequency(GPIO_MOTOR_PWM,   PWM_FREQ )
    self.pi.set_PWM_frequency(GPIO_STEERING_PWM,PWM_FREQ )

    # PWM発信停止
    self.pi.set_PWM_dutycycle(GPIO_MOTOR_PWM,	  0 )
    self.pi.set_PWM_dutycycle(GPIO_STEERING_PWM,	0 )

    # 制御用パラメータ初期化
    self.drive_steer  = Value("i", 0 )
    self.drive_motor  = Value("i", 0 )
    self.drive_active = Value("i", DRIVE_START )

    print( "init complete" )





#####################################################################
#	@fn		    __cruiseCntrol
#	@param		start_pt：ライン矩形の中心座標(x,y)
#	@param		vec     ：ライン矩形のベクトル
#	@brief		ライン矩形の形状からスピードとステアリングの制御を行う
#####################################################################
  def __cruiseCntrol( self, start_pt, vec ):

    #print(int(start_pt[0]), int(start_pt[1]))

    # 先端の矢印を描画
    angle = math.atan2(vec[1], vec[0])

    # ステアリングパラメータを取得
    if angle < 0:
        angle = math.pi + angle
    angle = angle-(math.pi/2)
    duty = int( angle*100 )

    # 中心軸からのズレをオフセットとして加算
    offset = int( start_pt[0] )- (CAM_SETTING_WIDTH//2)
    offset = offset//(CAM_SETTING_WIDTH//200)
    duty += offset

    print(duty)
    self.drive_steer.value = duty

    #ステアリングからスピードを算出（暫定）
    speed = 1000 - abs(duty)*4
    print(speed)

    self.drive_motor.value = speed





#####################################################################
#	@fn		    __drawAxis
#	@param		start_pt：ライン矩形の中心座標(x,y)
#	@param		vec     ：ライン矩形のベクトル
#	@param		colour  ：ベクトル描画色
#	@param		length  ：ベクトル長さ
#	@brief		ベクトルの描画を行う(デバッグ処理)
#####################################################################
  def __drawAxis( self, img, start_pt, vec, colour, length):

    # アンチエイリアス
    CV_AA = 16

    # 終了点
    end_pt = (int(start_pt[0] + length * vec[0]), int(start_pt[1] + length * vec[1]))

    # 中心を描画
    cv2.circle(img, (int(start_pt[0]), int(start_pt[1])), 5, colour, 1)
    print(int(start_pt[0]), int(start_pt[1]))

    # 軸線を描画
    cv2.line(img, (int(start_pt[0]), int(start_pt[1])), end_pt, colour, 1, CV_AA)

    # 先端の矢印を描画
    angle = math.atan2(vec[1], vec[0])

    # ラジアンを取得(0-314)
    duty = int( angle*100 )
    if duty < 0:
        duty = 314 - duty      
    duty = (duty-157)
    # ステアリングパラメータを決定
    self.drive_steer.value = duty

    qx0 = int(end_pt[0] - 9 * math.cos(angle + math.pi / 4));
    qy0 = int(end_pt[1] - 9 * math.sin(angle + math.pi / 4));
    cv2.line(img, end_pt, (qx0, qy0), colour, 1, CV_AA);

    qx1 = int(end_pt[0] - 9 * math.cos(angle - math.pi / 4));
    qy1 = int(end_pt[1] - 9 * math.sin(angle - math.pi / 4));
    cv2.line(img, end_pt, (qx1, qy1), colour, 1, CV_AA);





#####################################################################
#	@fn		    __get_rect
#	@param		contours：ラインの輪郭座標リスト
#	@brief		矩形の抽出を行う
#####################################################################
  def __get_rect( self, contours ):
    rect = cv2.minAreaRect(contours)
    box  = cv2.boxPoints(rect)
    box  = np.int0(box)
    return box





#####################################################################
#	@fn		    __find_lane
#	@param		box：矩形の四方の座標リスト
#	@brief		矩形からコースラインの検出/判定を行う
#####################################################################
  def __find_lane( self, box ):

    output   = 'find lane'
    position = list()
    location = 0

    if len(box) != 0:

      for j in range(len(box)):
        Horizontal = 'inside'
        Vertical   = 'inside'

        # 横座標検出
        if box[j][0] <= 0:
          #左側
          Horizontal = 'leftside'
        elif box[j][0] >= CAM_SETTING_WIDTH:
          #右側
          Horizontal = 'rightside'
        else:
          #内側
          Horizontal = 'inside'

        # 縦座標検出
        if box[j][1] <= 250:
          #上部
          Vertical = 'upperside'
        elif box[j][1] >= (CAM_SETTING_HEIGHT-10):# マージンを持つ
          #下部
          Vertical = 'lowerside'
        else:
          #内側
          Vertical = 'inside'

        position.append( [Horizontal, Vertical] )

        if Vertical == 'inside' and Horizontal == 'inside':
          location += 1

    if location >= 2:
      output = 'no lane'

    if output == 'no lane':
      for i in range(len(position)):
        if position[i][1] == 'lowerside':
          print("lowside!!")
          output   = 'find lane'
          break

    print(output)
    return output





#####################################################################
#	@fn		    __driving
#	@param		無し
#	@brief		ステアリングモーター/駆動モーターの制御を行うプロセス
#####################################################################
  def __driving(self):

    print( "drive start" )

    direction     = DIR_STOP
    now_speed     = 0
    now_steer     = STEERING_FORWORD

    while self.drive_active.value == DRIVE_START:  

      # 50ms周期で制御
      if 1:

        speed = self.drive_motor.value

        # モーター正逆制御
        if speed > 0:
          # 前進
          if direction != DIR_FORWARD and now_speed == 0:
            print("forward")
            direction = DIR_FORWARD
            self.pi.write( GPIO_MOTOR_CTRL1, 1 )
            self.pi.write( GPIO_MOTOR_CTRL2, 0 )
        elif speed < 0:
          # 後退
          if direction != DIR_BACK and now_speed == 0:
            print("back")
            direction = DIR_BACK
            self.pi.write( GPIO_MOTOR_CTRL1, 0 )
            self.pi.write( GPIO_MOTOR_CTRL2, 1 )
        elif speed == 0:
          # ブレーキ
          if direction != DIR_STOP:
            print("brake")
            direction = DIR_STOP
            self.pi.write( GPIO_MOTOR_CTRL1, 0 )
            self.pi.write( GPIO_MOTOR_CTRL2, 0 )

        # モーター速度調整
        speed = abs(speed//10)

        if now_speed < speed:
          now_speed = now_speed+1
          if now_speed > 100:
            now_speed = 100
          self.pi.set_PWM_dutycycle( GPIO_MOTOR_PWM, now_speed )

        elif now_speed > speed:
          now_speed = now_speed-3
          if now_speed < 0:
            now_speed = 0
          self.pi.set_PWM_dutycycle( GPIO_MOTOR_PWM, now_speed )


      # 1ms周期で制御
      if 1:
        
        # ステアリング動作量を取得
        duty  = STEERING_FORWORD + (self.drive_steer.value*2)      

        if duty < STEERING_LEFT:
          duty = STEERING_LEFT 
        elif duty > STEERING_RIGHT:
          duty = STEERING_RIGHT

        # ステアリングを制御
        if now_steer < duty:
          now_steer = now_steer + 1
          self.pi.set_servo_pulsewidth( GPIO_STEERING_PWM,	now_steer )
        elif now_steer > duty:
          now_steer = now_steer - 1
          self.pi.set_servo_pulsewidth( GPIO_STEERING_PWM,	now_steer )
      
      time.sleep(0.002)

    # 停止
    print( "__driving stop" )





#####################################################################
#	@fn		    __capture
#	@param		無し
#	@brief		カメラによるライン/物体検出と走行の制御を行う
#####################################################################
  def __capture(self):

    cap = cv2.VideoCapture(0)

    #カメラ初期設定
    cap.set(cv2.CAP_PROP_FPS, CAM_SETTING_FPS )
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_SETTING_WIDTH )
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_SETTING_HEIGHT)

    #モーターパワーオン
    self.pi.write( GPIO_MOTOR_STBY, 1 )

    while self.drive_active.value == DRIVE_START:

      # 画像を読み込む 
      ret, src = cap.read()

      # 台形変換
      pts1    = np.float32([[640,480],[385,150],[255,150],[0,480]])
      pts2    = np.float32([[640,480],[640,0],[0,0],[0,480]])
      M       = cv2.getPerspectiveTransform(pts1,pts2)
      dst     = cv2.warpPerspective(src,M,(640,480))

      # グレースケールに変換
      gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)

      # ２値化
      retval, bw = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

      # 白黒反転
      bw = cv2.bitwise_not(bw)

      # 輪郭を抽出
      img, contours, hierarchy = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

      if 1:
        # 取得した輪郭の中からレーンを判定する
        for i in range(0, len(contours)):

          # 輪郭の領域を計算
          area = cv2.contourArea(contours[i])

          # ノイズ（小さすぎる/大きすぎる領域）を除外
          if area < 1e4 or 1e5 < area:
            continue

          # 矩形からレーンを検出する
          box     = self.__get_rect( contours[i] )
          result  = self.__find_lane( box )

          # レーンのベクトルを計算する
          if result == 'find lane':
            #矩形描画
            cv2.drawContours( gray,[box],0,(0,0,255),2 )   

            # 輪郭データを浮動小数点型の配列に格納
            X = np.array(contours[i], dtype=np.float).reshape((contours[i].shape[0], contours[i].shape[2]))

            # PCA解析
            mean, eigenvectors = cv2.PCACompute(X, mean=np.array([], dtype=np.float), maxComponents=1)

            pt = (mean[0][0], mean[0][1])
            vec = (eigenvectors[0][0], eigenvectors[0][1])
            #self.__drawAxis( gray, pt, vec, (255, 255, 0), 150)
            self.__cruiseCntrol( pt, vec )
            break

      # 表示
      #cv2.imshow('input', src )
      cv2.imshow('output', gray )

      key = cv2.waitKey(1)
      if key != -1:
        self.drive_active.value = DRIVE_STOP

    #モーターストップ
    self.pi.write( GPIO_MOTOR_STBY, 0 )

    # 終了処理
    cap.release()
    cv2.destroyAllWindows()
    # 停止
    print("__capture stop")





#####################################################################
#	@fn		    __sixaxis
#	@param		無し
#	@brief		SIXAXISコントローラからの制御の管理プロセス
#####################################################################
  def __sixaxis(self):

    motor_driver = 0
    wait_counter = 0
    device_path  = '/dev/input/js0'


    with open(device_path, 'rb') as device:
      event = device.read( EVENT_SIZE )

      while True:
        if event:
          wait_counter = 0
        else: 
          if wait_counter < 120:
            wait_counter += 1
            print("sixaxis connecting wait...")
            time.sleep(1)
          else:
            print("timeout sixaxis connecting")
            break

        # イベント取得
        (ds3_time, ds3_val, ds3_type, ds3_num) = struct.unpack(EVENT_FORMAT, event)

        # プッシュボタン検出
        if ds3_type == 1:
          # select検出
          if ds3_num == 0:
            self.pi.write( GPIO_MOTOR_STBY, 0 )
            self.pi.stop()
            self.drive_active.value = DRIVE_STOP
            break			

          # start検出
          elif ds3_num == 3:
            if motor_driver == 0:
              motor_driver = 1
              print("motor driver on")

            self.pi.write( GPIO_MOTOR_STBY, motor_driver )

        # アナログセンサ検出
        elif ds3_type == 2:
          # Right Stick (horizontal)
          if ds3_num == 2:
            self.drive_steer.value = ds3_val//256
          # Left Stick (vartical)        
          elif ds3_num == 1:
            self.drive_motor.value = ds3_val//32

        event = device.read(EVENT_SIZE)

      # 停止
      print("__sixaxis stop")





#####################################################################
#	@fn		    Manual_ctrl
#	@param		無し
#	@brief		コントローラによるマニュアル走行制御メソッド
#####################################################################
  def Manual_ctrl(self):

    # 走行制御プロセスを生成
    driving = Process(target = self.__driving, args=() )
    driving.start()

    # SIXAXIS制御プロセスを生成
    sixaxis = Process(target = self.__sixaxis, args=() )
    sixaxis.start()

    driving.join()
    sixaxis.join()





#####################################################################
#	@fn		    Auto_ctrl
#	@param		無し
#	@brief		カメラ画像によるJ自動走行制御メソッド
#####################################################################
  def Auto_ctrl(self):

    # 走行制御プロセスを生成
    driving = Process(target = self.__driving, args=() )
    driving.start()

    # SIXAXIS制御プロセスを生成
    capture = Process(target = self.__capture, args=() )
    capture.start()

    driving.join()
    capture.join()





#####################################################################
#	@fn		    __main__
#	@param		無し
#	@brief		メイン処理
#####################################################################
if __name__ == '__main__':

  car = auto_cruise()

  #car.Manual_ctrl()
  car.Auto_ctrl()





