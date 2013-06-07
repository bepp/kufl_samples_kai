#include "num2str.h"

/******************************************************************************
** 関数	|	unsigned long num2str(long num, char str[], unsigned long offset)
** 概要	|	long型を文字列に変換する関数
** 引数	|	num		:	変換する数字(型が違う場合long型に変換する)
**		|	str		:	変換した文字列を格納する配列
**		|	offset	:	文字列のオフセット(通常0)
** 戻値	|	変換されて出来た文字列の文字数
*******************************************************************************	*/
extern unsigned long num2str(long num, char str[], unsigned long offset) {
	long sub_num=num;					// numのコピー
	long num_of_digits=1000000000;		// 桁
	unsigned long length=offset;		// 文字数
	unsigned char flag=0;

	/* 負の場合 */
	if(sub_num < 0) {
		str[length] = '-';
		sub_num *= -1;
		length++;
	}
	/* 文字列変換 */
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
