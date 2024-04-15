/*
 * lcm1602.h
 *
 *  Created on: Jun 9, 2022
 *      Author: J
 */

#ifndef INC_LCM1602_H_
#define INC_LCM1602_H_

#define LCD_ADDR (0x27 << 1)

#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)

#define LCD_DELAY_MS 5

void I2C_Scan();
void LCD_SendInternal(uint8_t lcd_addr, uint8_t data, uint8_t flags);
void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_Init(uint8_t lcd_addr);
void LCD_SendString(char *str);
void LCM1602_init();
void lcd_put_cur(int row, int col);


#endif /* INC_LCM1602_H_ */
