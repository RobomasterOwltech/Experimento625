/*
 * MotorPI.h
 *
 *  Created on: May 24, 2023
 *      Author: Fabian
 */

#ifndef KEYPAD_HPP_
#define KEYPAD_HPP_


#include "stm32h7xx_hal.h"
#include "main.h"
#include <string>

#define SLAVE_ADDRESS_LCD 0x4E
    
class Keypad {

private:
	char cmd;
	char data;
	char str;
	int row;
	int col;
	I2C_HandleTypeDef *hi2c1;

	char val_key;
	const char keys [4][4]={{'1','2','3','A'},
				            {'4','5','6','B'},
				            {'7','8','9','C'},
				            {'*','0','#','D'}};

	char password[4] = {'1','2','3','4'};
	char input[4];
	bool pressed;
public:
	
	Keypad(I2C_HandleTypeDef *_hi2c);
	char keypad_read();
	bool is_valid(char _key);
	bool check_password(char *input);

	void lcd_init (void);   
	void lcd_send_cmd (char cmd);  
	void lcd_send_data (char data);  
	void lcd_send_string (char *str);  
	void lcd_put_cur(int row, int col);
	void lcd_clear (void);	
};

#endif /* KEYPAD_HPP_ */
