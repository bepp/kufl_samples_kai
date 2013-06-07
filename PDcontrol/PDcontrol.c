/**
 ******************************************************************************
 **	�t�@�C���� : sample.c
 **
 **	�T�v : 2�֓|���U�q���C���g���[�X���{�b�g��TOPPERS/ATK1(OSEK)�pC�T���v���v���O����
 **	
 ** ���L : sample_c1 (���C���g���[�X���s�x�[�X�v���O����)
 ******************************************************************************
 **/
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h" /* �|���U�q����p�w�b�_�t�@�C�� */
#include "num2str.h"

/* ���L�̃}�N���͌�/���ɍ��킹�ĕύX����K�v������܂� */
/* sample_c1�}�N�� */
#define GYRO_OFFSET  605 /* �W���C���Z���T�I�t�Z�b�g�l(�p���x0[deg/sec]��) */
#define TAIL_ANGLE_STAND_UP 108 /* ���S��~���̊p�x[�x] */
#define delta_D 	4	/* �T�C�N���^�C�� */
#define TAIL_ANGLE_DRIVE      3 /* �o�����X���s���̊p�x[�x] */
#define P_GAIN             2.5F /* ���S��~�p���[�^������W�� */
#define PWM_ABS_MAX          60 /* ���S��~�p���[�^����PWM��΍ő�l */
#define e	50
#define DEVICE_NAME       "ET1"  /* Bluetooth�ʐM�p�f�o�C�X�� */
#define PASS_KEY          "1234" /* Bluetooth�ʐM�p�p�X�L�[ */

char tx_buf[BT_MAX_TX_BUF_SIZE] = "t\tv\tmA\tmB\tmC\tgyro\tsonar\tlight\ttouch\r\n\0";

//static void tail_control(signed int angle);
static void my_ecrobot_bt_data_logger();

//*****************************************************************************
// �֐��� : ecrobot_device_initialize
// ���� : �Ȃ�
// �߂�l	: �Ȃ�
// �T�v : ECROBOT�f�o�C�X�����������t�b�N�֐�
//*****************************************************************************
void ecrobot_device_initialize() {
	ecrobot_set_light_sensor_active(NXT_PORT_S3); /* ���Z���T�ԐFLED��ON */
	ecrobot_init_sonar_sensor(NXT_PORT_S2); /* �����g�Z���T(I2C�ʐM)�������� */
	nxt_motor_set_count(NXT_PORT_A, 0); /* ���S��~�p���[�^�G���R�[�_���Z�b�g */
	ecrobot_init_bt_slave(PASS_KEY); /* Bluetooth�ʐM������ */
}

//*****************************************************************************
// �֐��� : ecrobot_device_terminate
// ���� : �Ȃ�
// �߂�l	: �Ȃ�
// �T�v : ECROBOT�f�o�C�X�I�������t�b�N�֐�
//*****************************************************************************
void ecrobot_device_terminate() {
	ecrobot_set_light_sensor_inactive(NXT_PORT_S3); /* ���Z���T�ԐFLED��OFF */
	ecrobot_term_sonar_sensor(NXT_PORT_S2); /* �����g�Z���T(I2C�ʐM)���I�� */
	ecrobot_term_bt_connection(); /* Bluetooth�ʐM���I�� */
}

//*****************************************************************************
// �֐��� : user_1ms_isr_type2
// ���� : �Ȃ�
// �߂�l	: �Ȃ�
// �T�v : 1msec�������荞�݃t�b�N�֐�(OSEK ISR type2�J�e�S��)
//*****************************************************************************
void user_1ms_isr_type2(void){}

//*****************************************************************************
// �^�X�N�� : TaskMain
// �T�v : ���C���^�X�N
//*****************************************************************************
TASK(TaskMain) {
	signed char forward=0;      /* �O��i���� */
	signed char turn=0;         /* ���񖽗� */
	signed char pre_light_sensor=0;         /* �O�̌��Z���T�l */
	signed char now_light_sensor=0;         /* ���݂̌��Z���T�l */
	signed char pwm_L=0, pwm_R=0; /* ���E���[�^PWM�o�� */
	signed char control_P=0;	/* P����l */
	signed char const_P=0;	/* P����p�萔 */
	signed char control_D=0;	/* D����l */
	signed char const_D=0;	/* D����p�萔 */
	int white=0;		/* ���̂������l */
	int black=0;		/* ���̂������l */
	int gray=0;		/* �D�F�̂������l */
	//int gray_counter=0;		/* �D�F�̃J�E���^ */
	//int tail_counter=0;		/* �����ۂ̃J�E���^ */
	int log_counter=0;		/* ���O���p�J�E���^ */
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
		//tail_control(TAIL_ANGLE_STAND_UP); /* ���S��~�p�p�x�ɐ��� */

		if (ecrobot_get_touch_sensor(NXT_PORT_S4) == 1) {
			break; /* �^�b�`�Z���T�������ꂽ */
		}
	}

	balance_init();						/* �|���U�q���䏉���� */
	nxt_motor_set_count(NXT_PORT_C, 0); /* �����[�^�G���R�[�_���Z�b�g */
	nxt_motor_set_count(NXT_PORT_B, 0); /* �E���[�^�G���R�[�_���Z�b�g */

	while(1) {
		my_ecrobot_bt_data_logger();
//		if(tail_counter>100) {
//			tail_control(TAIL_ANGLE_DRIVE); /* �o�����X���s�p�p�x�ɐ��� */
//		}
//		else {
//			tail_counter++;
//		}
//
//		forward = 50; /* �O�i���� */
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
	
		/* �|���U�q����(forward = 0, turn = 0�ŐÎ~�o�����X) */
		balance_control(
			(float)forward,								 /* �O��i����(+:�O�i, -:��i) */
			(float)0,								 /* ���񖽗�(+:�E����, -:������) */
			(float)ecrobot_get_gyro_sensor(NXT_PORT_S1), /* �W���C���Z���T�l */
			(float)GYRO_OFFSET,							 /* �W���C���Z���T�I�t�Z�b�g�l */
			(float)nxt_motor_get_count(NXT_PORT_C),		 /* �����[�^��]�p�x[deg] */
			(float)nxt_motor_get_count(NXT_PORT_B),		 /* �E���[�^��]�p�x[deg] */
			(float)ecrobot_get_battery_voltage(),		 /* �o�b�e���d��[mV] */
			&pwm_L,										 /* �����[�^PWM�o�͒l */
			&pwm_R);									 /* �E���[�^PWM�o�͒l */
		nxt_motor_set_speed(NXT_PORT_C, pwm_L, 1); /* �����[�^PWM�o�̓Z�b�g(-100�`100) */
		nxt_motor_set_speed(NXT_PORT_B, pwm_R, 1); /* �E���[�^PWM�o�̓Z�b�g(-100�`100) */
		
		pre_light_sensor = now_light_sensor;

		systick_wait_ms(delta_D); /* 4msec�E�F�C�g */
	}
}

//*****************************************************************************
// �֐��� : tail_control
// ����  : angle (���[�^�ڕW�p�x[�x])
// �Ԃ�l : ����
// �T�v : ���s�̊��S��~�p���[�^�̊p�x����
//*****************************************************************************
/*
static void tail_control(signed int angle) {
	float pwm = (float)(angle - nxt_motor_get_count(NXT_PORT_A))*P_GAIN; ��ᐧ��
	PWM�o�͖O�a����
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
// �֐��� : my_ecrobot_bt_data_logger
// ����  : ����
// �Ԃ�l : ����
// �T�v : �f�[�^�̃��O���
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
	//tx_len++;			// �I�������͑���H

	ecrobot_send_bt(tx_buf, 0, tx_len);
}
