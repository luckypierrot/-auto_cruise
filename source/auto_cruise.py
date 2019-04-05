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
STEERING_LEFT        = 12000
STEERING_FORWORD     = 15000
STEERING_RIGHT       = 18000

# @brief	モーター速度定義
MAX_MOTOR_SPEED      = 500#345
MIN_MOTOR_SPEED      = 0

# @brief	進行方向定義
DIR_FORWARD          = 1
DIR_STOP             = 0
DIR_BACK             = -1

# @brief	車両の制御開始/停止
DRIVE_START          = 1
DRIVE_STOP           = 0

# @brief	カメラ設定値
CAM_SETTING_WIDTH    = 320
CAM_SETTING_HEIGHT   = 240
CAM_SETTING_FPS      = 24

# @brief 台形補正値
CAM_ADJUST_WIDTH     = 80   #左右から80pix
CAM_ADJUST_HEIGHT    = 150  #上から150px

# @brief ライン領域の無視判定スレッシュ
PIC_THRESH_MIN       = 1900
PIC_THRESH_MAX       = 15000

# @brief ライン角度-直進判定
STEER_ANGLE_MIN     = -30
STEER_ANGLE_MAX     = 30

# @brief ライン中心のずれ
LINE_GAP_MIN       = -80
LINE_GAP_MAX       = 80

# @brief ステアリング調整用ゲイン(floatで指定可能)
COEF_CENTERING     = 20.0
COEF_CENTERING_2   = 0.05
COEF_CENTERING_3   = 60.0

COEF_ANGLE         = 20.0
COEF_ANGLE_2       = 0.05
COEF_ANGLE_3       = 60.0

# @brief ステアリング現在値更新パラメータ
INCREMENT_COEF     = 0.05
INCREMENT_VAL      = 10

# @brief Drivingプロセススリープ時間(sec)
DRIVING_CYCLE      = 0.01

# @brief スタート信号検出の領域面積の閾値
GREEN_AREA_THRESH  = 10


#####################################################################
#	@class		auto_cruise
#	@brief		自動運転制御クラス
#####################################################################
class auto_cruise:

  #	@brief	privateパラメータ
  drive_steer          = 0
  drive_motor          = 0
  drive_active         = 0
  drive_centering      = 0
  sixaxis_switch       = 1
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
    self.drive_steer      = Value("i", 0 )
    self.drive_motor      = Value("i", 0 )
    self.drive_centering  = Value("i", 0 )
    self.drive_active     = Value("i", DRIVE_START )
    self.sixaxis_switch   = Value("i", 0 )

    #masuda
    self.drive_pre_steer  = Value("i", 0)

    print( "init complete" )





#####################################################################
#	@fn		    __cruiseCntrol
#	@param		start_pt：ライン矩形の中心座標(x,y)
#	@param		vec     ：ライン矩形のベクトル
#	@param		state   ：レーン検出状況
#	@brief		ライン矩形の形状からスピードとステアリングの制御を行う
#####################################################################
  def __cruiseCntrol( self, start_pt, vec, state ):

    angle = math.atan2(vec[1], vec[0])
    if angle < 0:
        angle = math.pi + angle
    angle = angle-(math.pi/2)

    # ラインの方向(ラジアン)からステアリングを算出(-157~157)
    duty    = int( angle*100 )

    # ライン中心座標の中央からの差分を算出(-160~160pix)
    offset  = int( start_pt[0] )- (CAM_SETTING_WIDTH//2)

    # モーター速度を設定
    speed   = MAX_MOTOR_SPEED

    # センターバリューの正負が同じ場合のみ、値を更新する
    determineSign = self.drive_centering.value*offset

    if state == 'find lane':
      self.drive_centering.value  = offset
      self.drive_steer.value      = duty
      self.drive_motor.value      = speed
      return True

    elif state == 'no lane' and determineSign >= 0:
      self.drive_centering.value  = offset
      self.drive_steer.value      = duty
      self.drive_motor.value      = speed
      return True

    else:
      print("i")
      return False





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
        ####################################
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
        ####################################
        if box[j][1] <= (CAM_SETTING_HEIGHT//2):
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
          output   = 'find lane'
          break

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
    dst_steer     = STEERING_FORWORD
    now_steer     = STEERING_FORWORD

    center        = 0
    i_center      = 0
    d_center      = 0

    duty          = 0
    i_duty        = 0
    d_duty        = 0

    while self.drive_active.value == DRIVE_START:  

      speed = self.drive_motor.value

      # モーター正逆制御
      ###############################
      if speed > 0:
        # 前進
        if direction != DIR_FORWARD and now_speed == 0:
          direction = DIR_FORWARD
          self.pi.write( GPIO_MOTOR_CTRL1, 1 )
          self.pi.write( GPIO_MOTOR_CTRL2, 0 )
      elif speed < 0:
        # 後退
        if direction != DIR_BACK and now_speed == 0:
          direction = DIR_BACK
          self.pi.write( GPIO_MOTOR_CTRL1, 0 )
          self.pi.write( GPIO_MOTOR_CTRL2, 1 )
      elif speed == 0:
        # ブレーキ
        if direction != DIR_STOP:
          direction = DIR_STOP
          self.pi.write( GPIO_MOTOR_CTRL1, 0 )
          self.pi.write( GPIO_MOTOR_CTRL2, 0 )


      # モーター速度制御
      ###############################
      speed = abs(speed)

      if now_speed < speed:
        #スピードアップ
        now_speed = now_speed+10
        if now_speed > MAX_MOTOR_SPEED:
          now_speed = MAX_MOTOR_SPEED
        self.pi.set_PWM_dutycycle( GPIO_MOTOR_PWM, now_speed//10 )

      elif now_speed > speed:
        #スピードダウン
        now_speed = now_speed-30
        if now_speed < MIN_MOTOR_SPEED:
          now_speed = MIN_MOTOR_SPEED
        self.pi.set_PWM_dutycycle( GPIO_MOTOR_PWM, now_speed//10 )


      # ステアリング-コントローラ制御
      ###############################
      if self.sixaxis_switch.value == 1:

        duty  = STEERING_FORWORD + (self.drive_steer.value*30) 

        if duty < STEERING_LEFT:
          duty = STEERING_LEFT 
        elif duty > STEERING_RIGHT:
          duty = STEERING_RIGHT

        # ステアリングを制御
        if now_steer < duty:
          now_steer = now_steer + 20
          self.pi.set_servo_pulsewidth( GPIO_STEERING_PWM,	now_steer//10 )
        elif now_steer > duty:
          now_steer = now_steer - 20
          self.pi.set_servo_pulsewidth( GPIO_STEERING_PWM,	now_steer//10 )

      # ステアリング-自動制御
      ###############################
      else:
        d_center  = center - self.drive_centering.value
        d_duty    = duty - self.drive_steer.value

        center    = self.drive_centering.value
        duty      = self.drive_steer.value

        #ラインが傾斜しているので、直交するように調整する
        if duty < STEER_ANGLE_MIN or STEER_ANGLE_MAX < duty:
          i_duty    +=  duty
        else:
          i_duty = 0

        #ラインの中心軸から離れているので、近づくように調整する
        if center < LINE_GAP_MIN or LINE_GAP_MAX < center:
          i_center  += center
        else:
          i_center = 0

        dst_steer =   STEERING_FORWORD

        dst_steer +=  int(duty*COEF_ANGLE)             #目標との差分がある場合に調整する
        dst_steer +=  int(i_duty*COEF_ANGLE_2)         #目標との差分に変化が無い場合に調整する
        dst_steer +=  int(d_duty*COEF_ANGLE_3)         #前回値から変化が大きい場合に調整する

        dst_steer +=  int(center*COEF_CENTERING)       #目標との差分がある場合に調整する
        dst_steer +=  int(i_center*COEF_CENTERING_2)   #目標との差分に変化が無い場合に調整する
        dst_steer +=  int(d_center*COEF_CENTERING_3)   #前回値から変化が大きい場合に調整する

        print( "duty",   int(duty*COEF_ANGLE),       "i_duty",   int(i_duty*COEF_ANGLE_2),       "d_duty",   int(d_duty*COEF_ANGLE_3)       )
        print( "center", int(center*COEF_CENTERING), "i_center", int(i_center*COEF_CENTERING_2), "d_center", int(d_center*COEF_CENTERING_3) )

        #現在値を目標に近づける
        if now_steer < dst_steer:
          param = dst_steer - now_steer
          now_steer += ( INCREMENT_VAL + int(param*INCREMENT_COEF) ) 
        elif now_steer > dst_steer:
          param = now_steer - dst_steer
          now_steer -= ( INCREMENT_VAL + int(param*INCREMENT_COEF) )

        #現在値を丸め込み
        if now_steer < STEERING_LEFT:
           now_steer = STEERING_LEFT 
        elif now_steer > STEERING_RIGHT:
          now_steer = STEERING_RIGHT

        self.pi.set_servo_pulsewidth( GPIO_STEERING_PWM,	now_steer//10 )

        print( "dst_steer =", dst_steer, "now_steer =", now_steer )

      time.sleep( DRIVING_CYCLE )

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

    x1 = ( CAM_ADJUST_WIDTH, CAM_ADJUST_HEIGHT )
    x2 = ( CAM_SETTING_WIDTH - CAM_ADJUST_WIDTH, CAM_ADJUST_HEIGHT )
    x3 = ( CAM_SETTING_WIDTH, CAM_SETTING_HEIGHT )
    x4 = ( 0, CAM_SETTING_HEIGHT )

    lane_state = 'find lane'

    #スタート信号の検出
    while self.drive_active.value == DRIVE_START: 
        ret, img = cap.read()

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower     = np.array([50, 100, 100])
        upper     = np.array([70, 255, 255])

        img_mask  = cv2.inRange( hsv, lower, upper )
        img_green = cv2.bitwise_and( img, img, mask=img_mask )

        img_gray  = cv2.cvtColor(img_green, cv2.COLOR_BGR2GRAY)
        size      = cv2.countNonZero(img_gray)
        print('green ', size)
        if size > GREEN_AREA_THRESH:
            self.pi.write( GPIO_MOTOR_STBY, 1 )
            print("motor driver on")
            break

    #ライン検出処理
    while self.drive_active.value == DRIVE_START:

      # 画像を読み込む 
      ret, src = cap.read()

      # ２値化
      lower     = np.array([0, 0, 0])
      upper     = np.array([100, 100, 100])
      gray      = cv2.inRange( src, lower, upper )

      # 台形変換
      pts1    = np.float32( [x1,x2,x3,x4] )
      pts2    = np.float32([[0,0],[CAM_SETTING_WIDTH,0],[CAM_SETTING_WIDTH,CAM_SETTING_HEIGHT],[0,CAM_SETTING_HEIGHT]])
      M       = cv2.getPerspectiveTransform(pts1,pts2)
      bw      = cv2.warpPerspective(gray,M,(CAM_SETTING_WIDTH, CAM_SETTING_HEIGHT))

      # 輪郭を抽出
      img, contours, hierarchy = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

      if self.sixaxis_switch.value == 0:
        result = 'no lane'

        # 取得した輪郭の中からレーンを判定する
        for i in range(0, len(contours)):

          # 輪郭の領域を計算
          area = cv2.contourArea(contours[i])

          # ノイズ（小さすぎる/大きすぎる領域）を除外
          if area < PIC_THRESH_MIN or PIC_THRESH_MAX < area:
            continue

          # 矩形からレーンを検出する
          box     = self.__get_rect( contours[i] )
          result  = self.__find_lane( box )

          # レーンのベクトルを計算する
          if result == 'find lane':
            #矩形描画
            #cv2.drawContours( bw,[box],0,(0,0,255),2 )   

            # 輪郭データを浮動小数点型の配列に格納
            X = np.array(contours[i], dtype=np.float).reshape((contours[i].shape[0], contours[i].shape[2]))

            # PCA解析
            mean, eigenvectors = cv2.PCACompute(X, mean=np.array([], dtype=np.float), maxComponents=1)

            pt  = (mean[0][0], mean[0][1])
            vec = (eigenvectors[0][0], eigenvectors[0][1])

            if self.__cruiseCntrol( pt, vec, lane_state ) == True:
              lane_state = 'find lane'

            break

        if result == 'no lane':
          lane_state = result

      # 表示
      cv2.imshow('output', bw )

      key = cv2.waitKey(1)

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

    device_path  = '/dev/input/js0'

    with open(device_path, 'rb') as device:
      event = device.read( EVENT_SIZE )

      while True:

        # イベント取得
        (ds3_time, ds3_val, ds3_type, ds3_num) = struct.unpack(EVENT_FORMAT, event)

        # プッシュボタン入力検出
        ############################
        if ds3_type == 1:

          # select検出
          if ds3_num == 0:
            ###########################
            # 各プロセスの停止、動作完了
            ###########################
            print("motor driver off")
            self.pi.write( GPIO_MOTOR_STBY, 0 )
            self.pi.stop()
            self.drive_active.value = DRIVE_STOP
            break

          # Start検出
          elif ds3_num == 3:
            #モータードライバ電源ON
            self.pi.write( GPIO_MOTOR_STBY, 1 )
            print("motor driver on")

          # R2検出
          elif ds3_num == 9:
            self.sixaxis_switch.value = 0
            print("sixaxis OFF")

          # R1検出
          elif ds3_num == 11:
            self.sixaxis_switch.value = 1
            self.drive_motor.value    = 0
            self.drive_steer.value    = 0
            print("sixaxis ON")


        # アナログ入力検出
        ############################
        elif ds3_type == 2:

          if self.sixaxis_switch.value == 1:

            # Right Stick (horizontal)
            if ds3_num == 2:
              if -32000 < ds3_val and ds3_val < 32000:
                self.drive_steer.value = ds3_val//25

            # Left Stick (vartical)        
            elif ds3_num == 1:
              if -32000 < ds3_val and ds3_val < 32000:
                self.drive_motor.value = -1*(ds3_val//32)

        event = device.read(EVENT_SIZE)

      # 停止
      print("__sixaxis stop")





#####################################################################
#	@fn		    Auto_ctrl
#	@param		無し
#	@brief		カメラ画像による自動走行制御メソッド
#####################################################################
  def Auto_ctrl(self):

    # SIXAXIS制御プロセスを生成
    sixaxis = Process(target = self.__sixaxis, args=() )
    sixaxis.start()

    # 走行制御プロセスを生成
    driving = Process(target = self.__driving, args=() )
    driving.start()

    # キャプチャ制御プロセスを生成
    capture = Process(target = self.__capture, args=() )
    capture.start()

    sixaxis.join()
    driving.join()
    capture.join()





#####################################################################
#	@fn		    __main__
#	@param		無し
#	@brief		メイン処理
#####################################################################
if __name__ == '__main__':

  car = auto_cruise()

  car.Auto_ctrl()





