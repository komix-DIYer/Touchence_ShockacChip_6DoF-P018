/* 
 * ショッカクチップ 6DoF-P018 のデータ取得プログラム
 * Fx, Fy, Fz, Mx, My, Mz, T;
 * 単純移動平均による平滑化
 * オフセット除去
 */

#include <Wire.h>

#define I2C_ADDR 0x01
#define DATA_NUM 7
#define BUFF_NUM 10

float CALIB[DATA_NUM] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
float D[DATA_NUM], D_buff[DATA_NUM][BUFF_NUM], D_sma[DATA_NUM], D_sum[DATA_NUM], D_off[DATA_NUM];
int off_cnt = -1, off_num = -1;

// センサデータ ( Fx, Fy, Fz, Mx, My, Mz, T ) の取得
void getFT()
{
  int temp;
  
  Wire.beginTransmission(I2C_ADDR); // Slave Address 7bit
  Wire.endTransmission(); // Recieve ACK
  
  // Recieve Data1 (Binary Counter, 14)
  Wire.requestFrom(I2C_ADDR, 1);
  if ( Wire.available() ) temp = Wire.read();
  
  // Recieve Data2~15
  if ( temp==14 )
  {
    Wire.requestFrom(I2C_ADDR, 14);
    
    for ( int i=0; i<14; i++ )
    {
      // Bind H&L bit of the dada
      if (Wire.available()) temp = Wire.read();
      temp = temp << 8;
      if (Wire.available()) temp = temp | Wire.read();
      
      switch (i)
      {
        case 0:
          D[0] = temp * CALIB[0] - D_off[0];
        case 2:
          D[1] = temp * CALIB[1] - D_off[1];
        case 4:
          D[2] = temp * CALIB[2] - D_off[2];
        case 6:
          D[3] = temp * CALIB[3] - D_off[3];
        case 8:
          D[4] = temp * CALIB[4] - D_off[4];
        case 10:
          D[5] = temp * CALIB[5] - D_off[5];
        case 12:
          D[6] = temp * CALIB[6] - D_off[6];
      }
      
      i++;
    }
  }
}

// センサデータの移動平均 (スパンは BUFF_NUM)
void calcSMA(void)
{
  for (int i = 0; i < DATA_NUM; i++)
  {
    for (int j = 0; j < BUFF_NUM - 1; j++)
      D_buff[i][j] = D_buff[i][j+1];

    D_buff[i][BUFF_NUM-1] = D[i];
  }
  
  for (int i = 0; i < DATA_NUM; i++)
  {
    float sma = 0;
    for (int j = 0; j < BUFF_NUM; j++)
      sma = sma + D_buff[i][j];
    
    D_sma[i] = sma/BUFF_NUM;
  }
}

// センサデータのオフセット量の算定 (スパンは off_num)
// off_num に正の値が与えられたとき実行される (普段は off_num = -1)
void getOffset()
{
  // 与えられた算定スパン off_num をカウンタ off_cnt にセット
  if ( off_num > 0 && off_cnt < 0 )
    off_cnt = off_num - 1;

  // カウンタが正である限りデータの移動平均値を積算 (カウンタはディクリメント)
  // カウンタが 0 のときオフセット量を確定 (積算値はリセット)
  // カウンタが負になった瞬間に off_num = -1 とする (この処理を一巡したら抜ける)
  if (off_cnt >= 0)
  { 
    for ( int i = 0; i < DATA_NUM; i++ )
    {
      D_sum[i] = D_sum[i] + D_sma[i];

      if (off_cnt == 0)
      {
        D_off[i] = D_off[i] + D_sum[i] / off_num;
        D_sum[i] = 0;
      }
    }

    off_cnt--;

    if ( off_cnt < 0)
      off_num = -1;
  }
  else // off_cnt < 0
  {
    off_num = -1;
  }
}

void setup()
{
  Wire.begin();
  //Wire.begin(26, 32); // for M5ATOM
  //Wire.begin(32, 33); // for M5StickCPlus
  
  Serial.begin(115200);
  while (!Serial);
  Serial.println();

  for (int i = 0; i < DATA_NUM; i++)
  {
    D_sma[i] = 0.0;
    D_off[i] = 0.0;
    
    for (int j = 0; j < BUFF_NUM; j++)
      D_buff[i][j] = 0.0;
  }
}

void loop()
{
  // オフセット量算定スパンの入力 (50~100)
  if ( Serial.available() > 0 )
  {
    String myString = Serial.readString();
    Serial.println(myString);
    off_num = myString.toInt();
    Serial.println(off_num);
  }
  
  getFT();
  calcSMA();
  getOffset();
  
//  Serial.print(Fx); Serial.print(", ");
//  Serial.print(Fy); Serial.print(", ");
//  Serial.print(Fz); Serial.print(", ");
//  Serial.print(Mx); Serial.print(", ");
//  Serial.print(My); Serial.print(", ");
//  Serial.print(Mz); Serial.print(", ");
//  Serial.print(T);
  
//  Serial.print(D[0]); Serial.print(", ");
//  Serial.print(D[1]); Serial.print(", ");
//  Serial.print(D[2]); Serial.print(", ");
//  Serial.print(D[3]); Serial.print(", ");
//  Serial.print(D[4]); Serial.print(", ");
//  Serial.print(D[5]); Serial.print(", ");
//  Serial.print(D[6]);
  
  Serial.print(D_sma[0]); Serial.print(", ");
  Serial.print(D_sma[1]); Serial.print(", ");
  Serial.print(D_sma[2]); Serial.print(", ");
  Serial.print(D_sma[3]); Serial.print(", ");
  Serial.print(D_sma[4]); Serial.print(", ");
  Serial.print(D_sma[5]); Serial.print(", ");
  Serial.print(D_sma[6]);
  
  Serial.println();
  
  delay(50);
}
