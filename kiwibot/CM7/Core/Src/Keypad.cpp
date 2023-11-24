/*
 * Joystick.cpp
 *
 *  Created on: Oct 31, 2023
 *      Author: serfa
 */

#include "Keypad.hpp"
//extern I2C_HandleTypeDef hi2c1;

Keypad::Keypad(I2C_HandleTypeDef *_hi2c)
{
	hi2c1 = _hi2c; 
}
void Keypad::lcd_send_cmd (char cmd)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}  
void Keypad::lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}
void Keypad::lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}
void Keypad::lcd_put_cur(int row, int col)
{
	switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd (col);
}
void Keypad::lcd_clear (void)
{
	lcd_send_cmd (0x80);
	for (int i=0; i<70; i++)
	{
		lcd_send_data (' ');
	}
}
void Keypad::lcd_init (void)
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);

  	// dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}   
char Keypad::keypad_read()
{
	
  HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);

  if(!(HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin))){
    HAL_Delay(4);
    while(!(HAL_GPIO_ReadPin (C1_GPIO_Port, C1_Pin)));
    HAL_Delay(4);
    val_key = keys[0][0];
  }
  if(!(HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin))){
    HAL_Delay(4);
    while(!(HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)));
    HAL_Delay(4);
    val_key = keys[0][1];
  }
  if(!(HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin))){
    HAL_Delay(4);
    while(!(HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)));
    HAL_Delay(4);
    val_key = keys[0][2];
  }
  if (!(HAL_GPIO_ReadPin(C4_GPIO_Port, C4_Pin))){
    HAL_Delay(4);
    while(!(HAL_GPIO_ReadPin(C4_GPIO_Port, C4_Pin)));
    HAL_Delay(4);
    val_key = keys[0][3];
  }

  HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);

  if(!(HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin))){
    HAL_Delay(4);
    while(!(HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)));
    HAL_Delay(4);
    val_key = keys[1][0];
  }
  if(!(HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin))){
    HAL_Delay(4);
    while(!(HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)));
    HAL_Delay(4);
    val_key = keys[1][1];
    }
  if(!(HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin))){
    HAL_Delay(4);
    while(!(HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)));
    HAL_Delay(4);
    val_key = keys[1][2];
  }
  if(!(HAL_GPIO_ReadPin(C4_GPIO_Port, C4_Pin))){
    HAL_Delay(4);
    while(!(HAL_GPIO_ReadPin(C4_GPIO_Port, C4_Pin)));
    HAL_Delay(4);
    val_key = keys[1][3];
  }

  HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);

  if(!(HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin))){
    HAL_Delay(4);
    while(!(HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)));
    HAL_Delay(4);
    val_key = keys[2][0];
  }
  if(!(HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin))){
    HAL_Delay(4);
    while(!(HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)));
    HAL_Delay(4);
    val_key = keys[2][1];
  }
  if(!(HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin))){
    HAL_Delay(4);
    while(!(HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)));
    HAL_Delay(4);
    val_key = keys[2][2];
  }
  if(!(HAL_GPIO_ReadPin (C4_GPIO_Port, C4_Pin))){
    HAL_Delay(4);
    while(!(HAL_GPIO_ReadPin(C4_GPIO_Port, C4_Pin)));
    HAL_Delay(4);
    val_key = keys[2][3];
  }

  HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_RESET);

  if(!(HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin))){
    HAL_Delay(4);
    while(!(HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin)));
    HAL_Delay(4);
    val_key = keys[3][0];
  }
  if(!(HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin))){
    HAL_Delay(4);
    while (!(HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin)));
    HAL_Delay(4);
    val_key = keys[3][1];
  }
  if(!(HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin))){
    HAL_Delay(4);
    while(!(HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin)));
    HAL_Delay(4);
    val_key = keys[3][2];
  }
  if(!(HAL_GPIO_ReadPin(C4_GPIO_Port, C4_Pin))){
    HAL_Delay(4);
    while(!(HAL_GPIO_ReadPin(C4_GPIO_Port, C4_Pin)));
    HAL_Delay(4);
    val_key = keys[3][3];
  }
  return val_key;
}

bool Keypad::is_valid(char _key)
{
    
  if(_key == '0'){
	  return true;
  }
  if(_key == '1'){
  	  return true;
  }
  if(_key == '2'){
  	  return true;
  }
  if(_key == '3'){
  	  return true;
  }
  if(_key == '4'){
  	  return true;
  }
  if(_key == '5'){
  	  return true;
  }
  if(_key == '6'){
	  return true;
  }
  if(_key == '7'){
	  return true;
  }
  if(_key == '8'){
	  return true;
  }
  if(_key == '9'){
	  return true;
  }
}
bool Keypad::check_password(char *_input)
{

  int aciertos = 0;
  for (int i = 0; i < 4; i++)
  {
    if (_input[i] == password[i])
    {
      aciertos++;
    }
  }

  if(aciertos == 4){
    return true;
  } else {
    return false;
  }
}
