/*
 * mpu_6050.c
 *
 *  Created on: May 9, 2024
 *      Author: angel
 */

/**** STEPS FOLLOWED  ************
1. Enable the I2C CLOCK and GPIO CLOCK
2. Configure the I2C PINs for ALternate Functions
	a) Select Alternate Function in MODER Register
	b) Select Open Drain Output
	c) Select High SPEED for the PINs
	d) Select Pull-up for both the Pins
	e) Configure the Alternate Function in AFR Register
3. Reset the I2C
4. Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
5. Configure the clock control registers
6. Configure the rise time register
7. Program the I2C_CR1 register to enable the peripheral
*/
#include "mpu_6050.h"
#include "stm32f4xx_hal.h"

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax = 0.0;
float Ay = 0.0;
float Az = 0.0;

float Gx = 0.0;
float Gy = 0.0;
float Gz = 0.0;

float Gx_offset = 0.0;
float Gy_offset = 0.0;
float Gz_offset = 0.0;

void I2C_Write(uint8_t data) {
	/**** STEPS FOLLOWED  ************
	1. Wait for the TXE (bit 7 in SR1) to set. This indicates that the DR is empty
	2. Send the DATA to the DR Register
	3. Wait for the BTF (bit 2 in SR1) to set. This indicates the end of LAST DATA transmission
	*/
	while (!(I2C1->SR1 & (1<< I2C_SR1_TXE_Pos)));  // wait for TXE bit to set
	I2C1->DR = data;
	while (!(I2C1->SR1 & (1<< I2C_SR1_BTF_Pos)));  // wait for BTF bit to set
}

void I2C_Start() {
	/**** STEPS FOLLOWED  ************
	1. Enable the ACK
	2. Send the START condition
	3. Wait for the SB ( Bit 0 in SR1) to set. This indicates that the start condition is generated
	*/
	I2C1->CR1 |= (1 << I2C_CR1_START_Pos);  // Generate START
	while(!(I2C1->SR1 & (1 << I2C_SR1_SB_Pos)));  // wait for ADDR bit to set
}

void I2C_Address(uint8_t address) {
	/**** STEPS FOLLOWED  ************
	1. Send the Slave Address to the DR Register
	2. Wait for the ADDR (bit 1 in SR1) to set. This indicates the end of address transmission
	3. clear the ADDR by reading the SR1 and SR2
	*/
	uint8_t temp = I2C1->SR1;
	I2C1->DR = address;  							//  send the address
	while(!(I2C1->SR1 & (1 << I2C_SR1_ADDR_Pos)));  // wait for ADDR bit to set
	temp = I2C1->SR1 | I2C1->SR2;  					// read SR1 and SR2 to clear the ADDR bit
}

void I2C_Read(uint8_t address, uint8_t *buffer, size_t size) {
	/**** STEPS FOLLOWED  ************
	1. If only 1 BYTE needs to be Read
		a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
		b) the Acknowledge disable is made during EV6 (before ADDR flag is cleared) and the STOP condition generation is made after EV6
		c) Wait for the RXNE (Receive Buffer not Empty) bit to set
		d) Read the data from the DR

	2. If Multiple BYTES needs to be read
	  a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
		b) Clear the ADDR bit by reading the SR1 and SR2 Registers
		c) Wait for the RXNE (Receive buffer not empty) bit to set
		d) Read the data from the DR
		e) Generate the Acknowledgment by setting the ACK (bit 10 in SR1)
		f) To generate the non-acknowledge pulse after the last received data byte, the ACK bit must be cleared just after reading the
			 second last data byte (after second last RxNE event)
		g) In order to generate the Stop/Restart condition, software must set the STOP/START bit
		   after reading the second last data byte (after the second last RxNE event)
	*/
	int remaining = size;

	/**** STEP 1 ****/
		if(size == 1) {
			/**** STEP 1-a ****/
			I2C1->DR = address;  //  send the address
			while(!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set

			/**** STEP 1-b ****/
			I2C1->CR1 &= ~(1<<10);  // clear the ACK bit
			uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit.... EV6 condition
			I2C1->CR1 |= (1<<9);  // Stop I2C

			/**** STEP 1-c ****/
			while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set

			/**** STEP 1-d ****/
			buffer[size-remaining] = I2C1->DR;  // Read the data from the DATA REGISTER
		}

	/**** STEP 2 ****/
		else {
			/**** STEP 2-a ****/
			I2C1->DR = address;  //  send the address
			while(!(I2C1->SR1 & (1 << I2C_SR1_ADDR_Pos)));  // wait for ADDR bit to set

			/**** STEP 2-b ****/
			uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit

			while(remaining > 2) {
				/**** STEP 2-c ****/
				while(!(I2C1->SR1 & (1 << I2C_SR1_RXNE_Pos)));  // wait for RxNE to set

				/**** STEP 2-d ****/
				buffer[size-remaining] = I2C1->DR;  // copy the data into the buffer

				/**** STEP 2-e ****/
				I2C1->CR1 |= 1 << I2C_CR1_ACK_Pos;  // Set the ACK bit to Acknowledge the data received

				remaining--;
			}

			// Read the SECOND LAST BYTE
			while(!(I2C1->SR1 & (1 << I2C_SR1_RXNE_Pos)));  // wait for RxNE to set
			buffer[size-remaining] = I2C1->DR;

			/**** STEP 2-f ****/
			I2C1->CR1 &= ~(1 << I2C_CR1_ACK_Pos);  // clear the ACK bit

			/**** STEP 2-g ****/
			I2C1->CR1 |= (1 << I2C_CR1_STOP_Pos);  // Stop I2C

			remaining--;

			// Read the Last BYTE
			while (!(I2C1->SR1 & (1 << I2C_SR1_RXNE_Pos)));  	// wait for RxNE to set
			buffer[size-remaining] = I2C1->DR;  // copy the data into the buffer
		}
}

void I2C_Stop() {
	/**** STEP 2-g ****/
	I2C1->CR1 |= (1 << I2C_CR1_STOP_Pos);  // Stop I2C
}

void I2C_Config() {
	RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOBEN_Pos);  // Enable GPIOB CLOCK

	GPIOB->MODER 	|= (2 << 16) | (2 << 18);  	// Bits (17:16)= 1:0 --> Alternate Function for Pin PB8;
	GPIOB->OTYPER 	|= (1 << 8)  | (1 << 9);  	// Bit8=1, Bit9=1  output open drain
	GPIOB->OSPEEDR 	|= (3 << 16) | (3 << 18);  	// Bits (17:16)= 1:1 --> High Speed for PIN PB8; Bits (19:18)= 1:1 --> High Speed for PIN PB9
	GPIOB->PUPDR 	|= (1 << 16) | (1 << 18);  	// Bits (17:16)= 0:1 --> Pull up for PIN PB8; Bits (19:18)= 0:1 --> pull up for PIN PB9
	GPIOB->AFR[1] 	|= (4 << 0)  | (4 << 4);  	// Bits (3:2:1:0) = 0:1:0:0 --> AF4 for pin PB8;  Bits (7:6:5:4) = 0:1:0:0 --> AF4 for pin PB9


	RCC->APB1ENR |= (1 << RCC_APB1ENR_I2C1EN_Pos);  // Enable I2C1 CLOCK

	I2C1->CR1 	|= (1 << I2C_CR1_SWRST_Pos);  	// reset the I2C
	I2C1->CR1 	&= ~(1 << I2C_CR1_SWRST_Pos);  	// Normal operation
	// Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
	I2C1->CR2 	|= (45 << 0);  					// PCLK1 FREQUENCY in MHz
	// Configure the clock control registers
	I2C1->CCR 	= 225 << 0;  					// computed as reported on github
	// Configure the rise time register
	I2C1->TRISE = 46;  							// computed as reported on github
	// Program the I2C_CR1 register to enable the peripheral
	I2C1->CR1 	|= (1 << I2C_CR1_ACK_Pos);  	// Enable the ACK
	I2C1->CR1 	|= (1 << 0);  	// Enable I2C
}

void MPU_Write(uint8_t address, uint8_t reg, uint8_t data)
{
	/**** STEPS FOLLOWED  ************
	1. START the I2C
	2. Send the ADDRESS of the Device
	3. Send the ADDRESS of the Register, where you want to write the data to
	4. Send the DATA
	5. STOP the I2C
	*/
	I2C_Start();
	I2C_Address(address);
	I2C_Write(reg);
	I2C_Write(data);
	I2C_Stop();
}

void MPU_Read(uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t size)
{
	/**** STEPS FOLLOWED  ************
	1. START the I2C
	2. Send the ADDRESS of the Device
	3. Send the ADDRESS of the Register, where you want to READ the data from
	4. Send the RESTART condition
	5. Send the Address (READ) of the device
	6. Read the data
	7. STOP the I2C
	*/
	I2C_Start();
	I2C_Address(address);
	I2C_Write(reg); 		// R/W 0
	I2C_Start();  			// repeated start
	I2C_Read(address + 0x01, buffer, size);
	I2C_Stop();
}

void MPU6050_Init(void)
{
	uint8_t check;
	uint8_t Data;

	I2C_Config();

	// check device ID WHO_AM_I

	MPU_Read(MPU_6050_ADDRESS, WHO_AM_I_REG, &check, 1);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		MPU_Write (MPU_6050_ADDRESS, PWR_MGMT_1_REG, Data);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		MPU_Write(MPU_6050_ADDRESS, SMPLRT_DIV_REG, Data);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ? 2g
		Data = 0x00;
		MPU_Write(MPU_6050_ADDRESS, ACCEL_CONFIG_REG, Data);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ? 250 ?/s
		Data = 0x00;
		MPU_Write(MPU_6050_ADDRESS, GYRO_CONFIG_REG, Data);
	}

}

void MPU6050_Read_Accel(void)
{

	uint8_t Rx_data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register
	for(int i = 0; i < 6; i++) {
		MPU_Read(MPU_6050_ADDRESS, ACCEL_XOUT_H_REG + i, &Rx_data[i], 1);
	}

	Accel_X_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data[1]);
	Accel_Y_RAW = (int16_t)(Rx_data[2] << 8 | Rx_data[3]);
	Accel_Z_RAW = (int16_t)(Rx_data[4] << 8 | Rx_data[5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;
}

void MPU6050_Read_Gyro(void)
{
	uint8_t Rx_data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register
	for(int i = 0; i < 6; i++){
		MPU_Read(MPU_6050_ADDRESS, GYRO_XOUT_H_REG + i, &Rx_data[i], 1);
	}

	Gyro_X_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data[1]);
	Gyro_Y_RAW = (int16_t)(Rx_data[2] << 8 | Rx_data[3]);
	Gyro_Z_RAW = (int16_t)(Rx_data[4] << 8 | Rx_data[5]);

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;
}

void MPU6050_Calc_Offset(void){
	int samples = 200;
	float sum_x = 0, sum_y = 0, sum_z = 0;
	for(int i = 0; i < samples; i++){
		MPU6050_Read_Gyro();
		sum_x += Gx;
		sum_y += Gy;
		sum_z += Gz;
		HAL_Delay(5);
	}
	Gx_offset = sum_x / samples;
	Gy_offset = sum_y / samples;
	Gz_offset = sum_z / samples;
}



