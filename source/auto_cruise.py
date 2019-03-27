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
MAX_MOTOR_SPEED      = 1000
MIN_MOTOR_SPEED      = 1000

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
PIC_THRESH_MIN       = 2000
PIC_THRESH_MAX       = 10000


# @brief ライン角度-直進判定
STEER_ANGLE_MIN     = -10    
STEER_ANGLE_MAX     = 10   

# @brief ライン中心のずれ
LINE_GAP_MIN       = -10
LINE_GAP_MAX       = 10

# @brief ステアリング調整用比例ゲイン(floatで指定可能)
COEF_CENTERING     = 0.1
COEF_ANGLE         = 0.1


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

    print( "init complete" )





#####################################################################
#	@fn		    __cruiseCntrol
#	@param		start_pt：ライン矩形の中心座標(x,y)
#	@param		vec     ：ライン矩形のベクトル
#	@brief		ライン矩形の形状からスピードとステアリングの制御を行う
#####################################################################
  def __cruiseCntrol( self, start_pt, vec ):

    angle = math.atan2(vec[1], vec[0])
    if angle < 0:
        angle = math.pi + angle
    angle = angle-(math.pi/2)

    # ラインの方向(ラジアン)からステアリングを算出(-157~157)
    duty    = int( angle*100 )

    # ライン中心座標の中央からの差分を算出(-160~160pix)
    offset  = int( start_pt[0] )- (CAM_SETTING_WIDTH//2)

    # モーター速度を算出
    speed   = MAX_MOTOR_SPEED - abs(duty)*4

    self.drive_centering.value  = offset
    self.drive_steer.value      = duty
    self.drive_motor.value      = speed





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
    now_steer     = STEERING_FORWORD

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
      if self.sixaxis_switch.value == 0:

        duty  = STEERING_FORWORD + (self.drive_steer.value*30) 

        if duty < STEERING_LEFT:
          duty = STEERING_LEFT 
        elif duty > STEERING_RIGHT:
          duty = STEERING_RIGHT

        # ステアリングを制御
        if now_steer < duty:
          now_steer = now_steer + 1
          self.pi.set_servo_pulsewidth( GPIO_STEERING_PWM,	now_steer//10 )
        elif now_steer > duty:
          now_steer = now_steer - 1
          self.pi.set_servo_pulsewidth( GPIO_STEERING_PWM,	now_steer//10 )

      # ステアリング-自動制御
      ###############################
      else:
        center  = self.drive_centering.value
        duty    = self.drive_steer.value

        #車体がラインに対して左右に曲がっている場合の調整
        if duty < STEER_ANGLE_MIN & STEER_ANGLE_MAX < duty:
          now_steer += int(duty*COEF_ANGLE)

        #車体がライン中心に対してずれている場合の調整
        if center < LINE_GAP_MIN & LINE_GAP_MAX < center:
          now_steer += int(center*COEF_CENTERING)

        #丸め込み
        if now_steer < STEERING_LEFT:
           now_steer = STEERING_LEFT 
        elif now_steer > STEERING_RIGHT:
          now_steer = STEERING_RIGHT

        self.pi.set_servo_pulsewidth( GPIO_STEERING_PWM,	now_steer//10 )

        print( (duty*COEF_ANGLE), self.drive_steer.value, now_steer)

      time.sleep(0.001)

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

    while self.drive_active.value == DRIVE_START:

      # 画像を読み込む 
      ret, src = cap.read()

      # グレースケールに変換
      gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

      # 台形変換
      pts1    = np.float32( [x1,x2,x3,x4] )
      pts2    = np.float32([[0,0],[CAM_SETTING_WIDTH,0],[CAM_SETTING_WIDTH,CAM_SETTING_HEIGHT],[0,CAM_SETTING_HEIGHT]])
      M       = cv2.getPerspectiveTransform(pts1,pts2)
      dst     = cv2.warpPerspective(gray,M,(CAM_SETTING_WIDTH, CAM_SETTING_HEIGHT))

      # ２値化
      retval, bw = cv2.threshold(dst, 10, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

      # 白黒反転
      bw = cv2.bitwise_not(bw)

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
            cv2.drawContours( dst,[box],0,(0,0,255),2 )   

            # 輪郭データを浮動小数点型の配列に格納
            X = np.array(contours[i], dtype=np.float).reshape((contours[i].shape[0], contours[i].shape[2]))

            # PCA解析
            mean, eigenvectors = cv2.PCACompute(X, mean=np.array([], dtype=np.float), maxComponents=1)

            pt  = (mean[0][0], mean[0][1])
            vec = (eigenvectors[0][0], eigenvectors[0][1])
            self.__cruiseCntrol( pt, vec )
            break

      # 表示
      #cv2.imshow('input', gray ) #
      cv2.imshow('output', dst )

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
              if -32000 < ds3_val & ds3_val < 32000:
                self.drive_steer.value = ds3_val//25

            # Left Stick (vartical)        
            elif ds3_num == 1:
              if -32000 < ds3_val & ds3_val < 32000:
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





