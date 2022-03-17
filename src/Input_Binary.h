#pragma once


void processBinInputs();

void InitBinInput1(uint8_t GPIO);
void InitBinInput2(uint8_t GPIO);
void InitBinInput3(uint8_t GPIO);
void InitBinInput4(uint8_t GPIO);

void input_BIN_1();
void input_BIN_2();
void input_BIN_3();
void input_BIN_4();

bool getStateInput1();
bool getStateInput2();
bool getStateInput3();
bool getStateInput4();