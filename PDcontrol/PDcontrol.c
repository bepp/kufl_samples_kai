/**
 ******************************************************************************
 **	ファイル名 : sample.c
 **
 **	概要 : 2輪倒立振子ライントレースロボットのTOPPERS/ATK1(OSEK)用Cサンプルプログラム
 **	
 ** 注記 : sample_c1 (ライントレース走行ベースプログラム)
 ******************************************************************************
 **/
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* 倒立振子制御用ヘッダファイル */
#include "num2str.h"

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
/* sample_c1マクロ */
#define GYRO_OFFSET  605 /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define TAIL_ANGLE_STAND_UP 108 /* 完全停止時の角度[度] */
#define delta_D 	4	/* サイクルタイム */
#define TAIL_ANGLE_DRIVE      3 /* バランス走行時の角度[度] */
#define P_GAIN             2.5F /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60 /* 完全停止用モータ制御PWM絶対最大値 */
#define e	50
#define DEVICE_NAME       "ET1"  /* Bluetooth通信用デバイス名 */
#define PASS_KEY          "1234" /* Bluetooth通信用パスキー */

char tx_buf[BT_MAX_TX_BUF_SIZE] = "t\tv\tmA\tmB\tmC\tgyro\tsonar\tlight\ttouch\r\n\0";

//static void tail_control(signed int angle);
static void my_ecrobot_bt_data_logger();

//*****************************************************************************
// 関数名 : ecrobot_device_initialize
// 引数 : なし
// 戻り値	: なし
// 概要 : ECROBOTデバイス初期化処理フック関数
//*****************************************************************************
void ecrobot_device_initialize() {
	ecrobot_set_light_sensor_active(NXT_PORT_S3); /* 光センサ赤色LEDをON */
	ecrobot_init_sonar_sensor(NXT_PORT_S2); /* 超音波センサ(I2C通信)を初期化 */
	nxt_motor_set_count(NXT_PORT_A, 0); /* 完全停止用モータエンコーダリセット */
	ecrobot_init_bt_slave(PASS_KEY); /* Bluetooth通信初期化 */
}

//*****************************************************************************
// 関数名 : ecrobot_device_terminate
// 引数 : なし
// 戻り値	: なし
// 概要 : ECROBOTデバイス終了処理フック関数
//*****************************************************************************
void ecrobot_device_terminate() {
	ecrobot_set_light_sensor_inactive(NXT_PORT_S3); /* 光センサ赤色LEDをOFF */
	ecrobot_term_sonar_sensor(NXT_PORT_S2); /* 超音波センサ(I2C通信)を終了 */
	ecrobot_term_bt_connection(); /* Bluetooth通信を終了 */
}

//*****************************************************************************
// 関数名 : user_1ms_isr_type2
// 引数 : なし
// 戻り値	: なし
// 概要 : 1msec周期割り込みフック関数(OSEK ISR type2カテゴリ)
//*****************************************************************************
void user_1ms_isr_type2(void){}

//*****************************************************************************
// タスク名 : TaskMain
// 概要 : メインタスク
//*****************************************************************************
TASK(TaskMain) {
	signed char forward=0;      /* 前後進命令 */
	signed char turn=0;         /* 旋回命令 */
	signed char pre_light_sensor=0;         /* 前の光センサ値 */
	signed char now_light_sensor=0;         /* 現在の光センサ値 */
	signed char pwm_L=0, pwm_R=0; /* 左右モータPWM出力 */
	signed char control_P=0;	/* P制御値 */
	signed char const_P=0;	/* P制御用定数 */
	signed char control_D=0;	/* D制御値 */
	signed char const_D=0;	/* D制御用定数 */
	int white=0;		/* 白のしきい値 */
	int black=0;		/* 黒のしきい値 */
	int gray=0;		/* 灰色のしきい値 */
	//int gray_counter=0;		/* 灰色のカウンタ */
	//int tail_counter=0;		/* しっぽのカウンタ */
	int log_counter=0;		/* ログ取り用カウンタ */
	//int tx_len=0;

	ecrobot_set_bt_device_name(DEVICE_NAME);

	ecrobot_send_bt(tx_buf, 0, 37);
	
	ecrobot_status_monitor("LET'S CARI!");
	while(1) {
		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1) {
			white = ecrobot_get_light_sensor(NXT_PORT_S3);
			ecrobot_status_monitor("white OK!");
			systick_wait_ms(100);
			break;
		}
	}

	while(1) {
		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 0) {
			break;
		}
	}

	while(1) {
		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1) {
			ecrobot_status_monitor("black OK!");
			black = ecrobot_get_light_sensor(NXT_PORT_S3);
			systick_wait_ms(100);
			break;
		}
	}
	
	while(1) {
		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 0) {
			break;
		}
	}

	while(1) {
		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1) {
			gray = ecrobot_get_light_sensor(NXT_PORT_S3);
			ecrobot_status_monitor("gray OK!");
			systick_wait_ms(100);
			break;
		}
	}

	while(1) {
		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 0) {
			break;
		}
	}

	while(1) {
		//tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */

		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1) {
			break; /* タッチセンサが押された */
		}
	}

	balance_init();						/* 倒立振子制御初期化 */
	nxt_motor_set_count(NXT_PORT_C, 0); /* 左モータエンコーダリセット */
	nxt_motor_set_count(NXT_PORT_B, 0); /* 右モータエンコーダリセット */

	while(1) {
		my_ecrobot_bt_data_logger();
//		if(tail_counter>100) {
//			tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */
//		}
//		else {
//			tail_counter++;
//		}
//
//		forward = 50; /* 前進命令 */
//		now_light_sensor = ecrobot_get_light_sensor(NXT_PORT_S3);
//
//		if(now_light_sensor >= gray - e && now_light_sensor <= gray + e) {
//			gray_counter++;
//		}
//		else if(gray_counter > 3) {
//			gray_counter--;
//		}
//
//		if(gray_counter >= 0) {
//			forward = turn = 0;
//		}
//		else {
			forward = 50;
			const_P = -1;
			const_D = -10;
			control_P = const_P * (now_light_sensor - (white + black)/2);
			control_D = const_D * (now_light_sensor - pre_light_sensor) / delta_D;
			turn = control_P + control_D;
			if(turn > 100) {
				turn = 100;
			}
			else if(turn < -100) {
				turn = -100;
			}
//		}
	
		/* 倒立振子制御(forward = 0, turn = 0で静止バランス) */
		balance_control(
			(float)forward,								 /* 前後進命令(+:前進, -:後進) */
			(float)0,								 /* 旋回命令(+:右旋回, -:左旋回) */
			(float)ecrobot_get_gyro_sensor(NXT_PORT_S1), /* ジャイロセンサ値 */
			(float)GYRO_OFFSET,							 /* ジャイロセンサオフセット値 */
			(float)nxt_motor_get_count(NXT_PORT_C),		 /* 左モータ回転角度[deg] */
			(float)nxt_motor_get_count(NXT_PORT_B),		 /* 右モータ回転角度[deg] */
			(float)ecrobot_get_battery_voltage(),		 /* バッテリ電圧[mV] */
			&pwm_L,										 /* 左モータPWM出力値 */
			&pwm_R);									 /* 右モータPWM出力値 */
		nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1); /* 左モータPWM出力セット(-100～100) */
		nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1); /* 右モータPWM出力セット(-100～100) */
		
		pre_light_sensor = now_light_sensor;

		systick_wait_ms(delta_D); /* 4msecウェイト */
	}
}

//*****************************************************************************
// 関数名 : tail_control
// 引数  : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
/*
static void tail_control(signed int angle) {
	float pwm = (float)(angle - nxt_motor_get_count(NXT_PORT_A))*P_GAIN; 比例制御
	PWM出力飽和処理
	if (pwm > PWM_ABS_MAX) {
		pwm = PWM_ABS_MAX;
	}
	else if (pwm < -PWM_ABS_MAX) {
		pwm = -PWM_ABS_MAX;
	}

	nxt_motor_set_speed(NXT_PORT_A, (signed char)pwm, 1);
}
*/
//*****************************************************************************
// 関数名 : my_ecrobot_bt_data_logger
// 引数  : 無し
// 返り値 : 無し
// 概要 : データのログ取り
//*****************************************************************************

void my_ecrobot_bt_data_logger() {
	unsigned long tx_len=0;

	U32 systick_time=0;
	U16	batery_voltage=0;
	int moter_count_PORT_A=0;
	int moter_count_PORT_B=0;
	int moter_count_PORT_C=0;
	U16 gyro_sensor=0;
	S32 sonar_sensor=0;
	U16 light_sensor=0;
	U8 touch_sensor=0;

	systick_time = systick_get_ms();
	batery_voltage = ecrobot_get_battery_voltage();
	moter_count_PORT_A = nxt_motor_get_count(NXT_PORT_A);
	moter_count_PORT_B = nxt_motor_get_count(NXT_PORT_B);
	moter_count_PORT_C = nxt_motor_get_count(NXT_PORT_C);
	gyro_sensor = ecrobot_get_gyro_sensor(NXT_PORT_S1);
	sonar_sensor = ecrobot_get_sonar_sensor(NXT_PORT_S2);
	light_sensor = ecrobot_get_light_sensor(NXT_PORT_S3);
	touch_sensor = ecrobot_get_touch_sensor(NXT_PORT_S4);

	tx_len = num2str(systick_time, tx_buf, tx_len);
	tx_buf[tx_len] = '\t';
	tx_len++;
	tx_len = num2str((long)batery_voltage, tx_buf, tx_len);
	tx_buf[tx_len] = '\t';
	tx_len++;
	tx_len = num2str((long)moter_count_PORT_A, tx_buf, tx_len);
	tx_buf[tx_len] = '\t';
	tx_len++;
	tx_len = num2str((long)moter_count_PORT_B, tx_buf, tx_len);
	tx_buf[tx_len] = '\t';
	tx_len++;
	tx_len = num2str((long)moter_count_PORT_C, tx_buf, tx_len);
	tx_buf[tx_len] = '\t';
	tx_len++;
	tx_len = num2str((long)gyro_sensor, tx_buf, tx_len);
	tx_buf[tx_len] = '\t';
	tx_len++;
	tx_len = num2str((long)sonar_sensor, tx_buf, tx_len);
	tx_buf[tx_len] = '\t';
	tx_len++;
	tx_len = num2str((long)light_sensor, tx_buf, tx_len);
	tx_buf[tx_len] = '\t';
	tx_len++;
	tx_len = num2str((long)touch_sensor, tx_buf, tx_len);
	tx_buf[tx_len] = '\r';
	tx_len++;
	tx_buf[tx_len] = '\n';
	tx_len++;
	tx_buf[tx_len] = '\0';
	//tx_len++;			// 終了文字は送る？

	ecrobot_send_bt(tx_buf, 0, tx_len);
}
