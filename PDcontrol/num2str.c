#include "num2str.h"

/******************************************************************************
** �֐�	|	unsigned long num2str(long num, char str[], unsigned long offset)
** �T�v	|	long�^�𕶎���ɕϊ�����֐�
** ����	|	num		:	�ϊ����鐔��(�^���Ⴄ�ꍇlong�^�ɕϊ�����)
**		|	str		:	�ϊ�������������i�[����z��
**		|	offset	:	������̃I�t�Z�b�g(�ʏ�0)
** �ߒl	|	�ϊ�����ďo����������̕�����
*******************************************************************************	*/
extern unsigned long num2str(long num, char str[], unsigned long offset) {
	long sub_num=num;					// num�̃R�s�[
	long num_of_digits=1000000000;		// ��
	unsigned long length=offset;		// ������
	unsigned char flag=0;

	/* ���̏ꍇ */
	if(sub_num < 0) {
		str[length] = '-';
		sub_num *= -1;
		length++;
	}
	/* ������ϊ� */
	while(num_of_digits > 0) {
		if(sub_num % num_of_digits != sub_num || flag == 1 || num_of_digits == 1) {
			str[length] = (char)(sub_num / num_of_digits + ASCII);
			sub_num %= num_of_digits;
			length++;
			flag = 1;
		}
		num_of_digits /= 10;
	}
	str[length] = '\0';

	return length;
}
