/*
 * ICS_library.ino
 * 
 * Arduino_STM32 with Kondo ICS servo library.
 * 
 * MIT License
 * 
 * version 0.50
 * 
 * 2019/9/5
 * 
 * devemin
 * https://github.com/devemin
 * https://twitter.com/devemin
 * 
 * 
 */


#include <libmaple/usart.h>
#include <libmaple/timer.h>

//定数
static const int ID_MIN = 0;
static const int ID_MAX = 31;
static const int ID_NUM = 32;

static const int EEPROM_NOTCHANGE = -4096;

static const int RETCODE_OK = 1;
static const int RETCODE_ERROR_ICSREAD = -1001;         //読み取りエラー、おそらくタイムアウト
static const int RETCODE_ERROR_ICSWRITE = -1002;        //返信コマンド内容エラー
static const int RETCODE_ERROR_IDWRONG = -1003;
static const int RETCODE_ERROR_OPTIONWRONG = -1004;
static const int RETCODE_ERROR_RETURNDATAWRONG = -1005;
static const int RETCODE_ERROR_EEPROMDATAWRONG = -1006;


static const int SERIAL1_BAUDRATE = 115200;
//プリメイドAI通常は115200。TeraTerm等でBluetooth RN42に接続でき、コマンド「$$$」→「D Enter確認」→「SU,92 Enter」→「D Enter確認」→電源入れ直し　で921600bpsの高速設定になります。
static const int SERIAL2_BAUDRATE = 1250000;
static const int SERIAL3_BAUDRATE = 1250000;

//デバッグ用
//プリメイドＡＩでは、Serial1がBluetooth RN42チップにつながっています。
//ＰＣとペアリングしTeratermで接続すれば、Serial1.print() (debugPrint())で文字列が送信でき表示されるはずです。
HardwareSerial *debugSer = &Serial1;
void debugPrint(String str) {
  debugSer->print(str);           //Serial1.print()によるデバッグ表示を無効にしたい場合はここをコメントアウトする
}

//IcsCommunicationクラスで用いるEEPROMデータ用構造体
struct EEPROMdata {
  int stretch = EEPROM_NOTCHANGE;
  int speed = EEPROM_NOTCHANGE;
  int punch = EEPROM_NOTCHANGE;
  int deadband = EEPROM_NOTCHANGE;
  int dumping = EEPROM_NOTCHANGE;
  int safetimer = EEPROM_NOTCHANGE;
  
  int flag_slave = EEPROM_NOTCHANGE;
  int flag_rotation = EEPROM_NOTCHANGE;
  int flag_pwminh = EEPROM_NOTCHANGE;
  int flag_free = EEPROM_NOTCHANGE;    //ここを変更しても書き込まない（ICSマネージャによると、読み出し参照のみとのこと）
  int flag_reverse = EEPROM_NOTCHANGE;
  
  int poslimithigh = EEPROM_NOTCHANGE;
  int poslimitlow = EEPROM_NOTCHANGE;
  int commspeed = EEPROM_NOTCHANGE;
  int temperaturelimit = EEPROM_NOTCHANGE;
  int currentlimit = EEPROM_NOTCHANGE;
  int response = EEPROM_NOTCHANGE;
  int offset = EEPROM_NOTCHANGE;
  int ID = EEPROM_NOTCHANGE;
  int charstretch1 = EEPROM_NOTCHANGE;
  int charstretch2 = EEPROM_NOTCHANGE;
  int charstretch3 = EEPROM_NOTCHANGE;

};


class IcsCommunication
{
  //パブリック変数
  public:

  //プライベート変数
  private:  
    
    static const int POS_MIN = 3500;
    static const int POS_MAX = 11500;

    static const int TIMEOUT_NORMAL = 5;
    static const int TIMEOUT_LONG = 600;          //ここのLONGタイムアウト値は、EEPROM書き込みエラーに関連してます。
                                                  //エラーが出る場合、型番によってはもう少し長くする必要があるかもしれません。

    static const int SC_CODE_EEPROM =  0x00;
    static const int SC_CODE_STRETCH = 0x01;
    static const int SC_CODE_SPEED =   0x02;
    static const int SC_CODE_CURRENT = 0x03;
    static const int SC_CODE_TEMPERATURE = 0x04;


    //デバッグ用
    unsigned long time1, time2, time3, time4;
        
    HardwareSerial *refSer ;
    struct usart_reg_map *regmap ;
    struct usart_dev *u_dev;
    uint32 baudrate = 115200;
    uint16 timeout;
    bool initHigh;

    int retcode;
    int retval;

  //パブリック関数
  public:
    IcsCommunication(HardwareSerial &ser);

    //初期化
    bool begin(uint32 brate=115200, uint16 timeoutnum=TIMEOUT_NORMAL, bool initFlag=true );
    void change_baudrate(uint32 brate);
    void change_timeout(uint16 timeoutnum);

    //サーボ移動関係
    int set_position(uint8 servolocalID, int val);      //サーボ動作する　　　　　位置が戻り値として来る
    int set_position_weak(uint8 servolocalID);          //サーボ脱力する　　　　　位置が戻り値として来る
    int set_position_weakandkeep(uint8 servolocalID);   //サーボ脱力後即動作する　位置が戻り値として来る

    //パラメータ関数系　電源切ると設定消える
    int get_stretch(uint8 servolocalID);
    int get_speed(uint8 servolocalID);
    int get_current(uint8 servolocalID);
    int get_temperature(uint8 servolocalID);
    
    int set_stretch(uint8 servolocalID, int val);
    int set_speed(uint8 servolocalID, int val);
    int set_currentlimit(uint8 servolocalID, int val);
    int set_temperaturelimit(uint8 servolocalID, int val);


    //EEPROM系　電源切っても設定消えない
    int get_EEPROM(uint8 servolocalID, EEPROMdata *r_edata);
    int set_EEPROM(uint8 servolocalID, EEPROMdata *w_edata);

    void show_EEPROMbuffer(byte *checkbuf);
    void show_EEPROMdata(EEPROMdata *edata);

    //標準のＩＤコマンド　ホストとサーボ１対１で使用する必要あり
    int get_ID();
    int set_ID(uint8 servolocalID);

    bool IsServoAlive(uint8 servolocalID);
        
  //プライベート関数
  private: 
    int transceive(byte *txbuf, byte *rxbuf, uint8 txsize, uint8 rxsize);
    int read_Param(uint8 servolocalID, byte sccode);
    int write_Param(uint8 servolocalID, byte sccode, int val);
    int read_EEPROMraw(uint8 servolocalID, byte *rxbuf);
    int check_EEPROMdata(EEPROMdata *edata);
    byte combine_2byte(byte a, byte b);
};


//コンストラクタ
IcsCommunication::IcsCommunication(HardwareSerial &ser) {

  refSer = &ser;
}


//初期化関数
//引数：　ボーレート(115200, 625000, 1250000)、タイムアウト値、500msecウェイト有無
//initHighで、500msecウェイトを最初に実行するかどうか決めます。
//（サーボ通信をシリアルのみとする手続き。EEPROMフラグに書き込み済なら不要）
//STM32 x Arduino は、接続方法によってSerialの対応番号が変わります。
//参考情報はこちら：　https://qiita.com/nanbuwks/items/5a01b924b192d5d36b31
//ST Linkで書き込み想定。　もしUSBシリアル接続で書き込んでいる場合には、Serial表記番号がずれます。メンドー
//どうしてもUSBシリアル接続の場合の方は、対応番号（Serial1,Serial2,Serial3の対応）を適当に書き替えてください。
bool IcsCommunication::begin(uint32 brate, uint16 timeoutnum, bool initFlag  ) {  
  baudrate = brate;
  timeout = timeoutnum;
  initHigh = initFlag; 
  
  uint8_t IcsPin;
      
  if (refSer == &Serial1) {
    IcsPin = PA9;
    regmap = USART1_BASE;
    u_dev = USART1;
  } else if (refSer == &Serial2) {
    IcsPin = PA2;
    regmap = USART2_BASE;
    u_dev = USART2;
  } else if (refSer == &Serial3) {
    IcsPin = PB10;
    regmap = USART3_BASE;
    u_dev = USART3;
  } else {
    return false;
  }
  
  if (initHigh) {
    //ICSサーボ設定　信号線を起動時500msec Highにすることで、シリアル通信になる（誤ってPWMにならないよう）
    //さらに心配なら、別途 EEPROMのPWMINHフラグを１にする事。
    digitalWrite(IcsPin, HIGH);
    delay(550);                 //長めに550msec.
    digitalWrite(IcsPin, LOW);
  }

  //ICSサーボ通信設定 (8bit, EVEN)
  refSer->begin(baudrate,SERIAL_8E1);
  refSer->setTimeout(timeout);

  //1-wire USART 用オープンドレイン設定　設定すると動かなかった、不要？
  //pinMode(IcsPin, OUTPUT_OPEN_DRAIN);

  //HDSELビットをオンにし、1-wire USARTを有効にする
  regmap->CR3 = regmap->CR3 | 0b00000000000000000000000000001000;

  return true;
}

//ボーレート変更関数　いつでも変更可能
//ICSでは115200, 625000, 1250000 のみ対応
void IcsCommunication::change_baudrate(uint32 brate) {
  
    usart_set_baud_rate(u_dev, USART_USE_PCLK, brate);
}


//タイムアウト値変更関数
void IcsCommunication::change_timeout(uint16 timeoutnum) {
  
  timeout = timeoutnum;
  refSer->setTimeout(timeout);
}


//基本的なサーボとのデータ送受信関数　すべてのベース
//引数：　送信バッファ、受信バッファ、送信サイズ、受信サイズ
int IcsCommunication::transceive(byte *txbuf, byte *rxbuf, uint8 txsize, uint8 rxsize) {
  
  int retLen;
  byte tmpbyte = txbuf[0];    //デバッグ用

  //TXビット　オン　/ RXビット　オフ
  regmap->CR1 = (regmap->CR1 & 0b111111111111111111111111111110011)  | 0b000000000000000000000000000001000;

  refSer->flush();
  time1 = micros();    //デバッグ用
  retLen = refSer->write(txbuf,txsize);
  time2 = micros();    //デバッグ用
  refSer->flush();

  //送信終わったのでtxbuf をゼロに（安全のため）
  for (int a=0; a<txsize; a++) {    
    txbuf[a]=0;
  }
    
  if (retLen != txsize) {
    //error
    return RETCODE_ERROR_ICSREAD;
  }

  //TXビット　オフ　/ RXビット　オン
  regmap->CR1 = (regmap->CR1 & 0b111111111111111111111111111110011)  | 0b000000000000000000000000000000100;

  //受信前なのでrxbuf をゼロに
  for (int a=0; a<rxsize; a++) {    
    rxbuf[a]=0;
  }

  //ここの空読みは、どうも必要だったり必要なかったり・・・
  //自分の回路の最初の環境では、これを抜くとループバックバイト列を余分に受信してしまうことがあった。
  //しかしプルアップ抵抗をしっかりブレッドボードにさしたら、逆にこれが邪魔でデータを消してしまった。
  //近藤科学に問い合わせたときは、プルアップ抵抗の値を調整してみてはどうでしょう、とのことだった。
  //プリメイドＡＩではとりあえず空読みしておく
  while (refSer->available() > 0)    //受信バッファの空読み
  {
    refSer->read();
  }    
  time3 = micros();    //デバッグ用
  retLen = refSer->readBytes(rxbuf,rxsize);
  time4 = micros();    //デバッグ用

  if (retLen != rxsize)
  {
    //error
    //debugPrint("RETCODE_ERROR_ICSWRITE: retLen: " + String(retLen) + ", rxsize: " + String(rxsize) + "\r\n");
    return RETCODE_ERROR_ICSWRITE;
  }

  //デバッグ用
  //debugPrint("transcieve: CMD " + String(tmpbyte,HEX) + ", " + String(time1) + ", " + String(time2) + ", " + String(time3) + ", " + String(time4) + ", " + "\r\n");
  
  return RETCODE_OK;
}




////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//サーボ制御系

//補足：サーボポジションの戻り値は、3500-11500範囲より少しだけ超える値が返ることもある(3481, 11541とか)

//サーボ位置移動
//引数：サーボＩＤ、ポジション値(3500-11500)
//戻り値に現在位置が入ります。
int IcsCommunication::set_position(uint8 servolocalID, int val) {
  int txsize = 3;   //この関数固有の、送信データバイト数
  int rxsize = 3;   //この関数固有の、受信データバイト数
  byte txbuf[3];
  byte rxbuf[3];

  for (int a=0; a<sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }
  for (int a=0; a<sizeof(rxbuf); a++) {
    rxbuf[a] = 0;
  }

  //引数チェック
  if ((servolocalID < ID_MIN) || (servolocalID > ID_MAX)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  if ((val < POS_MIN) || (val > POS_MAX)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  
  //送信データ作成
  txbuf[0] = 0x80 | servolocalID;
  txbuf[1] = val >> 7;
  txbuf[2] = val & 0b0000000001111111;

  //ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  //受信データ確認
  if (retcode == RETCODE_OK) {
    if ((rxbuf[0] & 0b01111111) == servolocalID) {
      retval = (rxbuf[1] << 7) + (rxbuf[2]);      
      return retval;
    } else {
      return RETCODE_ERROR_IDWRONG;
    }
  } else {
    //error
    return retcode;
  }
}



//サーボ脱力
//戻り値に現在位置が入ります。
int IcsCommunication::set_position_weak(uint8 servolocalID) {
  int txsize = 3;   //この関数固有の、送信データバイト数
  int rxsize = 3;   //この関数固有の、受信データバイト数
  byte txbuf[3];
  byte rxbuf[3];

  for (int a=0; a<sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }
  for (int a=0; a<sizeof(rxbuf); a++) {
    rxbuf[a] = 0;
  }
  
  //引数チェック
  if ((servolocalID < ID_MIN) || (servolocalID > ID_MAX)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }

  //送信データ作成
  txbuf[0] = 0x80 | servolocalID;
  txbuf[1] = 0;
  txbuf[2] = 0;

  //ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  //受信データ確認
  if (retcode == RETCODE_OK) {
    if ((rxbuf[0] & 0b01111111) == servolocalID) {
      retval = (rxbuf[1] << 7) + (rxbuf[2]);      
      return retval;
    } else {
      return RETCODE_ERROR_IDWRONG;
    }
  } else {
    //error
    return retcode;
  }
}


//脱力後、即時動作（現在位置確認用）
//戻り値に現在位置が入ります。
int IcsCommunication::set_position_weakandkeep(uint8 servolocalID) {
  int txsize = 3;   //この関数固有の、送信データバイト数
  int rxsize = 3;   //この関数固有の、受信データバイト数
  byte txbuf[3];
  byte rxbuf[3];

  for (int a=0; a<sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }
  for (int a=0; a<sizeof(rxbuf); a++) {
    rxbuf[a] = 0;
  }
  
  //引数チェック
  if ((servolocalID < ID_MIN) || (servolocalID > ID_MAX)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }

  //１回目の送信データ作成
  txbuf[0] = 0x80 | servolocalID;
  txbuf[1] = 0;
  txbuf[2] = 0;

  //ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  //受信データ確認
  if (retcode == RETCODE_OK) {
    if ((rxbuf[0] & 0b01111111) == servolocalID) {
      retval = (rxbuf[1] << 7) + (rxbuf[2]);
    
      //２回目の送信データ作成
      txbuf[0] = 0x80 | servolocalID;
      txbuf[1] = retval >> 7;
      txbuf[2] = retval & 0b0000000001111111;
    
      //２回目のICS送信
      retcode = transceive(txbuf, rxbuf, txsize, rxsize);
    
      //２回目の受信データ確認
      if (retcode == RETCODE_OK) {
        if ((rxbuf[0] & 0b01111111) == servolocalID) {
          retval = (rxbuf[1] << 7) + (rxbuf[2]);      
          return retval;
        } else {
          return RETCODE_ERROR_IDWRONG;
        }
      } else {
        //error
        return retcode;
      }
      
    } else {
      return RETCODE_ERROR_IDWRONG;
    }
  } else {
    //error
    return retcode;
  }
}



//EEPROM以外のパラメータ読み取りコマンド
//引数：サーボＩＤ、ポジション値(3500-11500)
//sccode は、SC_CODE_STRETCH, SC_CODE_SPEED, SC_CODE_CURRENT, SC_CODE_TEMPERATURE
//のどれかが入ります。
//SC_CODE_EEPROM はエラーとなります。EEPROM用の関数get_EEPROMを使ってください。
//戻り値に指定パラメータ値、またはエラーコード（負の値）が入ります。
int IcsCommunication::read_Param(uint8 servolocalID, byte sccode) {
  int txsize = 2;
  int rxsize = 3;
  byte txbuf[2];
  byte rxbuf[3];
  
  for (int a=0; a<sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }
  for (int a=0; a<sizeof(rxbuf); a++) {
    rxbuf[a] = 0;
  }
  
  if ( (sccode != SC_CODE_STRETCH) && (sccode != SC_CODE_SPEED) && (sccode != SC_CODE_CURRENT) && (sccode != SC_CODE_TEMPERATURE) ) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  
  //引数チェック
  if ((servolocalID < ID_MIN) || (servolocalID > ID_MAX)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  
  //送信データ作成
  txbuf[0] = 0xA0 | servolocalID;
  txbuf[1] = sccode;

  //ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  //受信データ確認
  if (retcode == RETCODE_OK) {
    if ( (rxbuf[0] == (0x20 | servolocalID)) && (rxbuf[1] == sccode) ) {
      //バッファチェック　ID, SC
      return rxbuf[2];        
    } else {
      return RETCODE_ERROR_RETURNDATAWRONG;
    }
  } else {
    //error
    return retcode;
  }
}


//EEPROM以外のパラメータ書き込みコマンド
//戻り値にRETCODE_OK（1の値）、またはエラーコード（負の値）が入ります。
//sccode は、
//SC_CODE_STRETCH, SC_CODE_SPEED, SC_CODE_CURRENT, SC_CODE_TEMPERATURE
//のどれかが入ります。
//SC_CODE_EEPROM はエラーとなります。EEPROM用の関数set_EEPROMを使ってください。
int IcsCommunication::write_Param(uint8 servolocalID, byte sccode, int val) {
  int txsize = 3;
  int rxsize = 3;
  byte txbuf[3];
  byte rxbuf[3];
  
  for (int a=0; a<sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }
  for (int a=0; a<sizeof(rxbuf); a++) {
    rxbuf[a] = 0;
  }
  
  if ( (sccode != SC_CODE_STRETCH) && (sccode != SC_CODE_SPEED) && (sccode != SC_CODE_CURRENT) && (sccode != SC_CODE_TEMPERATURE) ) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  
  //引数チェック
  if ((servolocalID < ID_MIN) || (servolocalID > ID_MAX)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  
  //送信データ作成
  txbuf[0] = 0xC0 | servolocalID;
  txbuf[1] = sccode;
  txbuf[2] = val;

  //ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  //受信データ確認
  if (retcode == RETCODE_OK) {
    if ( (rxbuf[0] == (0x40 | servolocalID)) && (rxbuf[1] == sccode) ) {
      //バッファチェック　ID, SC
      return RETCODE_OK;        
    } else {
      return RETCODE_ERROR_RETURNDATAWRONG;
    }
  } else {
    //error
    return retcode;
  }  
}




////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//パラメータ読み込み系

//全て、戻り値にパラメータ値、またはエラーコード（負の値）が入ります。

int IcsCommunication::get_stretch(uint8 servolocalID){
  byte sccode = SC_CODE_STRETCH;
  retval = read_Param(servolocalID, sccode);
  return retval;
}

int IcsCommunication::get_speed(uint8 servolocalID){
  byte sccode = SC_CODE_SPEED;
  retval = read_Param(servolocalID, sccode);
  return retval;
}

int IcsCommunication::get_current(uint8 servolocalID){
  byte sccode = SC_CODE_CURRENT;
  retval = read_Param(servolocalID, sccode);
  return retval;
}

int IcsCommunication::get_temperature(uint8 servolocalID){
  byte sccode = SC_CODE_TEMPERATURE;
  retval = read_Param(servolocalID, sccode);
  return retval;
}





////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//パラメータ書き込み系

//全て、戻り値にRETCODE_OK（1の値）、またはエラーコード（負の値）が入ります。

int IcsCommunication::set_stretch(uint8 servolocalID, int val){
  byte sccode = SC_CODE_STRETCH;
  if ((val < 1) || (val > 127) ) {
    return  RETCODE_ERROR_OPTIONWRONG;
  }
  retcode = write_Param(servolocalID, sccode, val);
  return retcode;
}

int IcsCommunication::set_speed(uint8 servolocalID, int val){
  byte sccode = SC_CODE_SPEED;
  if ((val < 1) || (val > 127) ) {
    return  RETCODE_ERROR_OPTIONWRONG;
  }
  retcode = write_Param(servolocalID, sccode, val);
  return retcode;
}

int IcsCommunication::set_currentlimit(uint8 servolocalID, int val){
  byte sccode = SC_CODE_CURRENT;
  if ((val < 1) || (val > 63) ) {
    return  RETCODE_ERROR_OPTIONWRONG;
  }
  retcode = write_Param(servolocalID, sccode, val);
  return retcode;
}

int IcsCommunication::set_temperaturelimit(uint8 servolocalID, int val){
  byte sccode = SC_CODE_TEMPERATURE;
  if ((val < 1) || (val > 127) ) {
    return  RETCODE_ERROR_OPTIONWRONG;
  }
  retcode = write_Param(servolocalID, sccode, val);
  return retcode;
}





////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
//EEPROM系

//EEPROM読み取り（生バイト）
//引数：　サーボＩＤ、受信バッファ
//通常は、次のget_EEPROM関数を使ってください。
//戻り値にRETCODE_OK（1の値）、またはエラーコード（負の値）が入ります。
int IcsCommunication::read_EEPROMraw(uint8 servolocalID, byte *rxbuf ) {
  int txsize = 2;
  int rxsize = 66;
  byte txbuf[2];

  for (int a=0; a<sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }

  byte sccode = SC_CODE_EEPROM;


  //引数チェック
  if ((servolocalID < ID_MIN) || (servolocalID > ID_MAX)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  
  //送信データ作成
  txbuf[0] = 0xA0 | servolocalID;
  txbuf[1] = sccode;

  //ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  //受信データ確認
  if (retcode != RETCODE_OK) {
    return retcode;
  }
  
  //バッファチェック　ID, SC, 0x5A
  if ( (rxbuf[0] != (0x20 | servolocalID)) || (rxbuf[1] != sccode) || (rxbuf[2] != 0x5) || (rxbuf[3] != 0xA) ) {
    //debugPrint("ID,SC,0x5A error.\r\n");
    return RETCODE_ERROR_RETURNDATAWRONG;
  }


  return RETCODE_OK;        
}



//EEPROM読み取り
//引数：　サーボＩＤ、ＥＥＰＲＯＭデータ構造体
//戻り値にRETCODE_OK（1の値）、またはエラーコード（負の値）が入ります。
int IcsCommunication::get_EEPROM(uint8 servolocalID, EEPROMdata *r_edata ) {
  byte rxbuf[66];

  retcode = read_EEPROMraw(servolocalID, rxbuf);
  
  //受信データ確認
  if (retcode != RETCODE_OK) {
    //debugPrint("get_EEPROM inside read_EEPROMraw error. retcode: " + String(retcode) + "\r\n");
    return retcode;
  }
  
  r_edata->stretch = combine_2byte(rxbuf[4],rxbuf[5]) / 2 ;   //2倍値で収納されてる
  r_edata->speed = combine_2byte(rxbuf[6],rxbuf[7]);
  r_edata->punch = combine_2byte(rxbuf[8],rxbuf[9]);
  r_edata->deadband = combine_2byte(rxbuf[10],rxbuf[11]);
  r_edata->dumping = combine_2byte(rxbuf[12],rxbuf[13]);
  r_edata->safetimer = combine_2byte(rxbuf[14],rxbuf[15]);
  
  r_edata->flag_slave =    (rxbuf[16]>>3) & 0b00000001;
  r_edata->flag_rotation = (rxbuf[16]   ) & 0b00000001;
  r_edata->flag_pwminh =   (rxbuf[17]>>3) & 0b00000001;
  r_edata->flag_free =     (rxbuf[17]>>1) & 0b00000001;
  r_edata->flag_reverse =  (rxbuf[17]   ) & 0b00000001;
  
  r_edata->poslimithigh = (combine_2byte(rxbuf[18],rxbuf[19]) << 8) | combine_2byte(rxbuf[20],rxbuf[21]);
  r_edata->poslimitlow = (combine_2byte(rxbuf[22],rxbuf[23]) << 8) | combine_2byte(rxbuf[24],rxbuf[25]);

  int commflagval = combine_2byte(rxbuf[28],rxbuf[29]);
  if (commflagval == 0x00) {
    r_edata->commspeed = 1250000;  
  } else if (commflagval == 0x01) {
    r_edata->commspeed = 625000;  
  } else if (commflagval == 0x0A) {
    r_edata->commspeed = 115200;  
  } else {
    r_edata->commspeed = EEPROM_NOTCHANGE;  
  }
    

  
  r_edata->temperaturelimit = combine_2byte(rxbuf[30],rxbuf[31]);
  r_edata->currentlimit = combine_2byte(rxbuf[32],rxbuf[33]);
  r_edata->response = combine_2byte(rxbuf[52],rxbuf[53]);

  byte tmpoffset = combine_2byte(rxbuf[54],rxbuf[55]);
  //ICSマネージャ挙動では、正負反対に収納されているのでそれに合わせる
  if ((tmpoffset >> 7) == 0b00000001) {   //負ビット
    r_edata->offset = ( ( ~(tmpoffset) & 0b01111111 ) + 1 );
  } else {                                      //正ビット
    r_edata->offset = 0 - tmpoffset;
  }

  r_edata->ID = combine_2byte(rxbuf[58],rxbuf[59]);

  r_edata->charstretch1 = combine_2byte(rxbuf[60],rxbuf[61]) / 2;   //2倍値で収納されてる
  r_edata->charstretch2 = combine_2byte(rxbuf[62],rxbuf[63]) / 2;   //2倍値で収納されてる
  r_edata->charstretch3 = combine_2byte(rxbuf[64],rxbuf[65]) / 2;   //2倍値で収納されてる


  retcode = check_EEPROMdata(r_edata);

  if (retcode != RETCODE_OK) {
    //debugPrint("get_EEPROM inside check_EEPROMdata error. retcode: " + String(retcode) + "\r\n");
    return retcode;
  }
  
  return RETCODE_OK;       
}



//(beta test)
//EEPROM書き込み
//引数：　サーボＩＤ、ＥＥＰＲＯＭデータ構造体
//戻り値にRETCODE_OK（1の値）、またはエラーコード（負の値）が入ります。
//書き込みコマンドは時間がややかかるようで、この関数内のみ別途タイムアウト時間を長くしています。
//https://twitter.com/devemin/status/1165875204419010561
int IcsCommunication::set_EEPROM(uint8 servolocalID, EEPROMdata *w_edata){
  //w_edata 内のデータのうち、EEPROM_NOTCHANGE でないものだけ書き込む
  int txsize = 66;  //処理2つ目のEEPROM書き込みでのサイズ
  int rxsize = 2;   //処理2つ目のEEPROM書き込みでのサイズ
  byte txbuf[66];   //処理2つ目のEEPROM書き込みでのサイズ
  byte rxbuf[66];   //処理1つ目のEEPROM読み取り、処理2つ目のEEPROM書き込み、の両方で使うバッファなのでsize:66としてる
  
  byte sccode = SC_CODE_EEPROM;

  for (int a=0; a<sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }
  for (int a=0; a<sizeof(rxbuf); a++) {
    rxbuf[a] = 0;
  }

  //引数チェック
  if ((servolocalID < ID_MIN) || (servolocalID > ID_MAX)) {
    return RETCODE_ERROR_OPTIONWRONG;
  }
  //変更禁止部分以外の、設定値の正当性チェック
  retcode = check_EEPROMdata(w_edata);
  if (retcode != RETCODE_OK) {
    return  RETCODE_ERROR_EEPROMDATAWRONG;
  }

  //////////////////////////////////////////////////////////////////////
  //関数内1つ目の処理　EEPROM読み取り
  //書き込む前に必ず最新のEEPROM読み取り！
  retcode = read_EEPROMraw(servolocalID, rxbuf);
  if (retcode != RETCODE_OK) {
    return  retcode;
  }
  
  //EEPROMデータ先頭の0x5Aチェック
  int checkdata = combine_2byte(rxbuf[2],rxbuf[3]);
  if ( checkdata != 0x5A ) {
    return  RETCODE_ERROR_EEPROMDATAWRONG;  
  }

  //////////////////////////////////////////////////////////////////////
  //関数内2つ目の処理　EEPROM書き込み
  //EEPROMに書き込むデータの準備
  for (int a=0; a<txsize; a++) {    
    txbuf[a]=rxbuf[a];      //元々の値を全てにコピー。この後、必要な項目のみ変更する
  }

  //送信データ先頭の作成
  txbuf[0] = 0xC0 | servolocalID;
  txbuf[1] = sccode;

  //送信バッファtxbuf に、引数で受け取ったEEPROMデータの変更部分のみコピーする
  if (w_edata->stretch != EEPROM_NOTCHANGE ) {
    txbuf[4] = (byte)(w_edata->stretch*2) >>4;          //2倍値で収納されてるので処理
    txbuf[5] = (byte)(w_edata->stretch*2) & 0b0000000000001111;
  }
  
  if (w_edata->speed != EEPROM_NOTCHANGE ) { 
    txbuf[6] = (byte)(w_edata->speed) >>4;
    txbuf[7] = (byte)(w_edata->speed) & 0b0000000000001111;    
  }
  if (w_edata->punch != EEPROM_NOTCHANGE ) { 
    txbuf[8] = (byte)(w_edata->punch) >>4;
    txbuf[9] = (byte)(w_edata->punch) & 0b0000000000001111;    
  }
  if (w_edata->deadband != EEPROM_NOTCHANGE ) { 
    txbuf[10] = (byte)(w_edata->deadband) >>4;
    txbuf[11] = (byte)(w_edata->deadband) & 0b0000000000001111;    
  }
  if (w_edata->dumping != EEPROM_NOTCHANGE ) { 
    txbuf[12] = (byte)(w_edata->dumping) >>4;
    txbuf[13] = (byte)(w_edata->dumping) & 0b0000000000001111;    
  }
  if (w_edata->safetimer != EEPROM_NOTCHANGE ) { 
    txbuf[14] = (byte)(w_edata->safetimer) >>4;
    txbuf[15] = (byte)(w_edata->safetimer) & 0b0000000000001111;    
  }

  if (w_edata->flag_slave != EEPROM_NOTCHANGE ) { 
    if (w_edata->flag_slave == 1) { txbuf[16] = txbuf[16] | 0b00001000 ; }
    if (w_edata->flag_slave == 0) { txbuf[16] = txbuf[16] & 0b11110111 ; }
  }
  if (w_edata->flag_rotation != EEPROM_NOTCHANGE ) { 
    if (w_edata->flag_rotation == 1) { txbuf[16] = txbuf[16] | 0b00000001 ; }
    if (w_edata->flag_rotation == 0) { txbuf[16] = txbuf[16] & 0b11111110 ; }
  }
  if (w_edata->flag_pwminh != EEPROM_NOTCHANGE ) { 
    if (w_edata->flag_pwminh == 1) { txbuf[17] = txbuf[17] | 0b00001000 ; }
    if (w_edata->flag_pwminh == 0) { txbuf[17] = txbuf[17] & 0b11110111 ; }
  }
  if (w_edata->flag_free != EEPROM_NOTCHANGE ) { 
    //freeフラグは参照のみとのこと。ここを書き込む事は、マニュアル等に説明は無いので挙動不明
    //if (w_edata->flag_free == 1) { txbuf[17] = txbuf[17] | 0b00000010 ; }
    //if (w_edata->flag_free == 0) { txbuf[17] = txbuf[17] & 0b11111101 ; }
  }
  if (w_edata->flag_reverse != EEPROM_NOTCHANGE ) { 
    if (w_edata->flag_reverse == 1) { txbuf[17] = txbuf[17] | 0b00000001 ; }
    if (w_edata->flag_reverse == 0) { txbuf[17] = txbuf[17] & 0b11111110 ; }
  }


  if (w_edata->poslimithigh != EEPROM_NOTCHANGE ) { 
    txbuf[18] = (w_edata->poslimithigh >> 12) & 0b00000000000000000000000000001111;
    txbuf[19] = (w_edata->poslimithigh >> 8 ) & 0b00000000000000000000000000001111;
    txbuf[20] = (w_edata->poslimithigh >> 4 ) & 0b00000000000000000000000000001111;
    txbuf[21] = (w_edata->poslimithigh      ) & 0b00000000000000000000000000001111;
  }
  
  if (w_edata->poslimitlow != EEPROM_NOTCHANGE ) { 
    txbuf[22] = (w_edata->poslimitlow >> 12) & 0b00000000000000000000000000001111;
    txbuf[23] = (w_edata->poslimitlow >> 8 ) & 0b00000000000000000000000000001111;
    txbuf[24] = (w_edata->poslimitlow >> 4 ) & 0b00000000000000000000000000001111;
    txbuf[25] = (w_edata->poslimitlow      ) & 0b00000000000000000000000000001111;
  }
  

  if (w_edata->commspeed != EEPROM_NOTCHANGE ) { 
    if (w_edata->commspeed == 115200) {
      txbuf[28] = 0x0;
      txbuf[29] = 0xA;
    } else if (w_edata->commspeed == 625000) {
      txbuf[28] = 0x0;
      txbuf[29] = 0x1;
    } else if (w_edata->commspeed == 1250000) {
      txbuf[28] = 0x0;
      txbuf[29] = 0x0;
    }  
  }

  if (w_edata->temperaturelimit != EEPROM_NOTCHANGE ) { 
    txbuf[30] = (byte)(w_edata->temperaturelimit) >>4;
    txbuf[31] = (byte)(w_edata->temperaturelimit) & 0b0000000000001111;    
  }
  if (w_edata->currentlimit != EEPROM_NOTCHANGE ) { 
    txbuf[32] = (byte)(w_edata->currentlimit) >>4;
    txbuf[33] = (byte)(w_edata->currentlimit) & 0b0000000000001111;    
  }
  if (w_edata->response != EEPROM_NOTCHANGE ) { 
    txbuf[52] = (byte)(w_edata->response) >>4;
    txbuf[53] = (byte)(w_edata->response) & 0b0000000000001111;    
  }
  if (w_edata->offset != EEPROM_NOTCHANGE ) {        //ICSマネージャ挙動では、正負反対に収納されているのでそれに合わせる
    byte tmpoffset = ~(w_edata->offset) + 1;
    txbuf[54] = (byte)(tmpoffset) >>4;
    txbuf[55] = (byte)(tmpoffset) & 0b0000000000001111;    
  }
  if (w_edata->ID != EEPROM_NOTCHANGE ) { 
    //IDは専用のコマンドがあるので、本来はそちらで行う。こちらから書き込んでも私の環境ではID書き換えできているが、リファレンスマニュアルには記載無し
    txbuf[58] = (byte)(w_edata->ID) >>4;
    txbuf[59] = (byte)(w_edata->ID) & 0b0000000000001111;    
  }

  if (w_edata->charstretch1 != EEPROM_NOTCHANGE ) {
    txbuf[60] = (byte)(w_edata->charstretch1*2) >>4;          //2倍値で収納されてるので処理
    txbuf[61] = (byte)(w_edata->charstretch1*2) & 0b0000000000001111;
  }
  if (w_edata->charstretch2 != EEPROM_NOTCHANGE ) {
    txbuf[62] = (byte)(w_edata->charstretch2*2) >>4;          //2倍値で収納されてるので処理
    txbuf[63] = (byte)(w_edata->charstretch2*2) & 0b0000000000001111;
  }
  if (w_edata->charstretch1 != EEPROM_NOTCHANGE ) {
    txbuf[64] = (byte)(w_edata->charstretch3*2) >>4;          //2倍値で収納されてるので処理
    txbuf[65] = (byte)(w_edata->charstretch3*2) & 0b0000000000001111;
  }


  change_timeout(TIMEOUT_LONG); //タイムアウトを長くする（重要！）
  
  //ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  change_timeout(TIMEOUT_NORMAL); //タイムアウトを戻す（重要！）

  
  if (retcode != RETCODE_OK) {
    //debugPrint("error: " + String(retcode));
    return retcode;
  }  
  
  //受信データ確認
  //バッファチェック　ID, SC
  if ( (rxbuf[0] != (0x40 | servolocalID)) || (rxbuf[1] != sccode) ) {
    //debugPrint("return data is incorrect.\r\n");
    return RETCODE_ERROR_RETURNDATAWRONG;
  }
  
  return RETCODE_OK;   

    
}


//データがEEPROM用として正当かどうかを確認する関数
//元々のサーボ内の変更禁止バイト部分の読み込み等は別で行っている
//内容がEEPROM_NOTCHANGE（初期状態）だと、書き込み不適と判断しエラーを返す
int IcsCommunication::check_EEPROMdata(EEPROMdata *edata) {
  int checkdata;
  int tmpretcode= RETCODE_OK;
  
  checkdata = edata->stretch;
  if ( (checkdata != EEPROM_NOTCHANGE) && (checkdata < 1) || (checkdata > 127) ) {
    debugPrint("EEPROMdata error: stretch\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }
  checkdata = edata->speed;
  if ( (checkdata != EEPROM_NOTCHANGE) &&  (checkdata < 0x01) || (checkdata > 0x7F) ) {
    debugPrint("EEPROMdata error: speed\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }
  checkdata = edata->punch;
  if ( (checkdata != EEPROM_NOTCHANGE) &&  (checkdata < 0x00) || (checkdata > 0x0A) ) {
    debugPrint("EEPROMdata error: punch\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG; 
  } 
  checkdata = edata->deadband;
  if ( (checkdata != EEPROM_NOTCHANGE) &&  (checkdata < 0x00) || (checkdata > 0x10) ) {
    debugPrint("EEPROMdata error: deadband\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }
  checkdata = edata->dumping;
  if ( (checkdata != EEPROM_NOTCHANGE) &&  (checkdata < 0x01) || (checkdata > 0xFF) ) {
    debugPrint("EEPROMdata error: dumping\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }
  checkdata = edata->safetimer;
  if ( (checkdata != EEPROM_NOTCHANGE) &&  (checkdata < 0x01) || (checkdata > 0xFF) ) {
    debugPrint("EEPROMdata error: safetimer\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }

  //フラグ
  if ( (edata->flag_slave != EEPROM_NOTCHANGE) &&  (edata->flag_slave != 0) && (edata->flag_slave != 1) ) {
    debugPrint("EEPROMdata error: flag_slave\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }
  if ( (edata->flag_rotation != EEPROM_NOTCHANGE) &&  (edata->flag_rotation != 0) && (edata->flag_rotation != 1) ) {
    debugPrint("EEPROMdata error: flag_rotation\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }
  if ( (edata->flag_pwminh != EEPROM_NOTCHANGE) &&  (edata->flag_pwminh != 0) && (edata->flag_pwminh != 1) ) {
    debugPrint("EEPROMdata error: flag_pwminh\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }
  if ( (edata->flag_free != EEPROM_NOTCHANGE) &&  (edata->flag_free != 0) && (edata->flag_free != 1) ) {
    debugPrint("EEPROMdata error: flag_free\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }
  if ( (edata->flag_reverse != EEPROM_NOTCHANGE) &&  (edata->flag_reverse != 0) && (edata->flag_reverse != 1) ) {
    debugPrint("EEPROMdata error: flag_reverse\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }

  
  checkdata = edata->poslimithigh;
  if ( (checkdata != EEPROM_NOTCHANGE) &&  (checkdata < 8000) || (checkdata > POS_MAX) ) {
    debugPrint("EEPROMdata error: poslimithigh\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }
  checkdata = edata->poslimitlow;
  if ( (checkdata != EEPROM_NOTCHANGE) &&  (checkdata < POS_MIN) || (checkdata > 7000) ) {
    debugPrint("EEPROMdata error: poslimitlow\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }

  if ( (edata->commspeed != EEPROM_NOTCHANGE) && (edata->commspeed != 115200) && (edata->commspeed != 625000) && (edata->commspeed != 1250000) ) {
    debugPrint("EEPROMdata error: commspeed\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }
  
  checkdata = edata->temperaturelimit;
  if ( (checkdata != EEPROM_NOTCHANGE) &&  (checkdata < 0x01) || (checkdata > 0x7F) ) {
    debugPrint("EEPROMdata error: temperaturelimit\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }
  checkdata = edata->currentlimit;
  if ( (checkdata != EEPROM_NOTCHANGE) &&  (checkdata < 0x01) || (checkdata > 0x3F) ) {
    debugPrint("EEPROMdata error: currentlimit\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }
  checkdata = edata->response;
  if ( (checkdata != EEPROM_NOTCHANGE) &&  (checkdata < 0x01) || (checkdata > 0x05) ) {
    debugPrint("EEPROMdata error: response\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }
  checkdata = edata->offset;
  if ( (checkdata != EEPROM_NOTCHANGE) &&  (checkdata < -128) || (checkdata > 127) ) {
    debugPrint("EEPROMdata error: offset\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }  
  checkdata = edata->ID;
  if ( (checkdata != EEPROM_NOTCHANGE) &&  (checkdata < 0x00) || (checkdata > 0x1F) ) {
    debugPrint("EEPROMdata error: ID\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }
  checkdata = edata->charstretch1;
  if ( (checkdata != EEPROM_NOTCHANGE) &&  (checkdata < 1) || (checkdata > 127) ) {
    debugPrint("EEPROMdata error: charstretch1\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }
  checkdata = edata->charstretch2;
  if ( (checkdata != EEPROM_NOTCHANGE) &&  (checkdata < 1) || (checkdata > 127) ) {
    debugPrint("EEPROMdata error: charstretch2\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }
  checkdata = edata->charstretch3;
  if ( (checkdata != EEPROM_NOTCHANGE) &&  (checkdata < 1) || (checkdata > 127) ) {
    debugPrint("EEPROMdata error: charstretch3\r\n");
    tmpretcode = RETCODE_ERROR_EEPROMDATAWRONG;  
  }


  return tmpretcode;
}


//上位下位4bitずつに分かれた2byteを、1byteにして取得する関数
byte IcsCommunication::combine_2byte(byte a, byte b){
  return (byte)((a<<4) | b);
}


//バッファ内のEEPROM生バイトデータを表示する関数
void IcsCommunication::show_EEPROMbuffer(byte *checkbuf) {
  
  //////////////////////////////////////////////////////////////////
  //テストデバッグコード
  debugPrint("//////////////////////////////////////////////\r\n");
  
  debugPrint(String(checkbuf[0],HEX) + " " );
  debugPrint(String(checkbuf[1],HEX) + " " );
  debugPrint("\r\n" );
  for (int c=0; c<=63; c++) {       
    debugPrint(String(checkbuf[c+2],HEX) + " " );
    if ((c%8) == 7) {
      debugPrint("\r\n" );
    }
  }
  debugPrint("\r\n" );
  //////////////////////////////////////////////////////////////////
  
}



//EEPROMデータの表示関数
void IcsCommunication::show_EEPROMdata(EEPROMdata *edata) {

  debugPrint("===================================================\r\n");
  debugPrint("----EEPROM data------------------------------------\r\n");
  
  debugPrint("stretch:          " + String(edata->stretch) + "[0x" + String(edata->stretch, HEX) + "] (1-127)\r\n");
  debugPrint("speed:            " + String(edata->speed) + "[0x" + String(edata->speed, HEX) + "] (1-127)\r\n");
  debugPrint("punch:            " + String(edata->punch) + "[0x" + String(edata->punch, HEX) + "] (0-10)\r\n");
  debugPrint("deadband:         " + String(edata->deadband) + "[0x" + String(edata->deadband, HEX) + "] (0-5)\r\n");
  debugPrint("dumping:          " + String(edata->dumping) + "[0x" + String(edata->dumping, HEX) + "] (1-255)\r\n");
  debugPrint("safetimer:        " + String(edata->safetimer) + "[0x" + String(edata->safetimer, HEX) + "] (10-255)\r\n");
  
  debugPrint("flag_slave:       " + String(edata->flag_slave) + " (0/1)\r\n");
  debugPrint("flag_rotation:    " + String(edata->flag_rotation) + " (0/1)\r\n");
  debugPrint("flag_pwminh:      " + String(edata->flag_pwminh) + " (0/1)\r\n");
  debugPrint("flag_free:        " + String(edata->flag_free) + " (0/1)\r\n");
  debugPrint("flag_reverse:     " + String(edata->flag_reverse) + " (0/1)\r\n");
  
  debugPrint("poslimithigh:     " + String(edata->poslimithigh) + "[0x" + String(edata->poslimithigh, HEX) + "] (8000-11500)\r\n");
  debugPrint("poslimitlow:      " + String(edata->poslimitlow) + "[0x" + String(edata->poslimitlow, HEX) + "] (3500-7000)\r\n");
  debugPrint("commspeed:        " + String(edata->commspeed) + " (115200/625000/1250000)\r\n");
  debugPrint("temperaturelimit: " + String(edata->temperaturelimit) + "[0x" + String(edata->temperaturelimit, HEX) + "] (1-127)\r\n");
  debugPrint("currentlimit:     " + String(edata->currentlimit) + "[0x" + String(edata->currentlimit, HEX) + "] (1-63)\r\n");
  debugPrint("response:         " + String(edata->response) + "[0x" + String(edata->response, HEX) + "] (1-5)\r\n");
  debugPrint("offset:           " + String(edata->offset) + "[0x" + String(edata->offset, HEX) + "] (-127-127)\r\n");
  debugPrint("ID:               " + String(edata->ID) + "[0x" + String(edata->ID, HEX) + "] (0-31)\r\n");
  debugPrint("charstretch1:     " + String(edata->charstretch1) + "[0x" + String(edata->charstretch1, HEX) + "] (1-127)\r\n");
  debugPrint("charstretch2:     " + String(edata->charstretch2) + "[0x" + String(edata->charstretch2, HEX) + "] (1-127)\r\n");
  debugPrint("charstretch3:     " + String(edata->charstretch3) + "[0x" + String(edata->charstretch3, HEX) + "] (1-127)\r\n");
  debugPrint("\r\n");
  debugPrint("    -4096: this mean NO_CHANGE.                    \r\n");
  debugPrint("---------------------------------------------------\r\n");
  debugPrint("===================================================\r\n");
  debugPrint("\r\n");

}



//サーボの存在を確認
//EEPROMからID読み込みを繰り返して、判断する。
//EEPROM読み取りは各種データチェックが入ってるので、その処理を複数回乗り越えてれば、
//おそらくそのIDは本物だろう、という考え。
bool IcsCommunication::IsServoAlive(uint8 servolocalID) {
  EEPROMdata tmped;
  int repnum = 10;    //チェック回数
  
  for (int a=0; a<repnum; a++) {
    retcode = get_EEPROM(servolocalID, &tmped);
    if ( (retcode != RETCODE_OK) || (servolocalID != tmped.ID) ) {
      //debugPrint(String(servolocalID) + "," + String(tmped.ID) + "\r\n");
      return false;
    }
  }

  return true;
}




//このID読み込みコマンドは、標準のものだが、ホストーサーボを１対１で接続して使用するもの。
//もし複数接続していた場合は、返信IDは不正なデータとなるので信用性がない。
int IcsCommunication::get_ID() {
  int txsize = 4;   //この関数固有の、送信データバイト数
  int rxsize = 1;   //この関数固有の、受信データバイト数
  byte txbuf[4];
  byte rxbuf[66];   //重複IDチェックのため多バイト確保

  for (int a=0; a<sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }
  for (int a=0; a<sizeof(rxbuf); a++) {
    rxbuf[a] = 0;
  }

  //送信データ作成
  txbuf[0] = 0xFF;
  txbuf[1] = 0x00;
  txbuf[2] = 0x00;
  txbuf[3] = 0x00;

  //ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  //受信データ確認
  if (retcode == RETCODE_OK) {
    if ((rxbuf[0] >> 5) == 0b00000111) {
      retval = (rxbuf[0] & 0b00011111);      
      return retval;
    } else {
      return RETCODE_ERROR_IDWRONG;
    }
  } else {
    //error
    return retcode;
  }
  
}


//このID書き込みコマンドは、標準のものだが、ホストーサーボを１対１で接続して使用するもの。
//もし複数接続していた場合は、全てのサーボが同じIDに書き換わってしまうので注意。
//もし該当サーボのIDが既にわかってるのであれば、こちらを使わずEEPROM書き替え関数の方でID指定して書き替えられます。
//20μsec 程で返信コマンドは来るが、再度書き込みする場合は、
//EEPROM書き込みと同等時間待たないとエラーとなり返信来ないので、別途delay()が必要
//KRS-4031HV で500μsec 程
//https://twitter.com/devemin/status/1165865232318775296
int IcsCommunication::set_ID(uint8 servolocalID) {
  int txsize = 4;   //この関数固有の、送信データバイト数
  int rxsize = 1;   //この関数固有の、受信データバイト数
  byte txbuf[4];
  byte rxbuf[66];   //重複IDチェックのため多バイト確保

  for (int a=0; a<sizeof(txbuf); a++) {
    txbuf[a] = 0;
  }
  for (int a=0; a<sizeof(rxbuf); a++) {
    rxbuf[a] = 0;
  }

  //送信データ作成
  txbuf[0] = 0xE0 | servolocalID;
  txbuf[1] = 0x01;
  txbuf[2] = 0x01;
  txbuf[3] = 0x01;

  //ICS送信
  retcode = transceive(txbuf, rxbuf, txsize, rxsize);

  //受信データ確認
  if (retcode == RETCODE_OK) {
    if ( ((rxbuf[0] >> 5) == 0b00000111) && ( (rxbuf[0] & 0b00011111) == servolocalID ) ) {
      retval = (rxbuf[0] & 0b00011111);      
      return retval;
    } else {
      return RETCODE_ERROR_IDWRONG;
    }
  } else {
    //error
    return retcode;
  }
  
}






IcsCommunication ics2(Serial2); //HV
IcsCommunication ics3(Serial3); //MV



void setup() {
  debugSer->begin(SERIAL1_BAUDRATE);
  
  ics2.begin(SERIAL2_BAUDRATE, 50, true);
  ics3.begin(SERIAL3_BAUDRATE, 50, true);
  
}


void loop() {

  /*
  //全EEPROM読み込み
  EEPROMdata tested;
  int endid = 11;
  for (int a=1; a<=endid; a++) {
    int retcode = ics3.get_EEPROM(a, &tested);
    if (retcode != RETCODE_OK) {
      debugPrint("error.\r\n");   
      delay(200);
      break;
    }
    debugPrint(String(a,DEC) + ",");
    debugPrint(String(tested.stretch,DEC) + ",");
    debugPrint(String(tested.speed,DEC) + ",");
    debugPrint(String(tested.punch,DEC) + ",");
    debugPrint(String(tested.deadband,DEC) + ",");
    debugPrint(String(tested.dumping,DEC) + ",");
    debugPrint(String(tested.safetimer,DEC) + ",");
    debugPrint(String(tested.flag_slave,DEC) + ",");
    debugPrint(String(tested.flag_rotation,DEC) + ",");
    debugPrint(String(tested.flag_pwminh,DEC) + ",");
    debugPrint(String(tested.flag_free,DEC) + ",");
    debugPrint(String(tested.flag_reverse,DEC) + ",");
    debugPrint(String(tested.poslimithigh,DEC) + ",");
    debugPrint(String(tested.poslimitlow,DEC) + ",");
    debugPrint(String(tested.commspeed,DEC) + ",");
    debugPrint(String(tested.temperaturelimit,DEC) + ",");
    debugPrint(String(tested.currentlimit,DEC) + ",");
    debugPrint(String(tested.response,DEC) + ",");
    debugPrint(String(tested.offset,DEC) + ",");
    debugPrint(String(tested.ID,DEC) + ",");
    debugPrint(String(tested.charstretch1,DEC) + ",");
    debugPrint(String(tested.charstretch2,DEC) + ",");
    debugPrint(String(tested.charstretch3,DEC) + ",");
    debugPrint("\r\n");

    delay(200);
  }
  delay(2000);  
*/




/*
  int retval;
  //ポジション値送信・戻り値　テスト
  for (int b=10; b<=11; b++) {       
    retval = ics3.set_position(b, 7500);
    debugPrint(String(retval) + "\r\n" );
  }
  delay(1000);
  for (int b=10; b<=11; b++) {       
    retval = ics3.set_position(b, 8000);
    debugPrint(String(retval) + "\r\n" );
  }
  delay(1000);
*/


/*
  //サーボ脱力　テスト
  int retpos;  
  for (int b=10; b<=11; b++) {       
    retpos = ics3.set_position(b, 7500);
    debugPrint(String(retpos) + "\r\n" );
    delay(1000);
 
    retpos = ics3.set_position(b, 8000);
    debugPrint(String(retpos) + "\r\n" );
    delay(1000);

    retpos = ics3.set_position_weak(b);
    debugPrint(String(retpos) + "\r\n" );
    delay(1000);
  }  
*/


/*
  //サーボ脱力アンドキープ　テスト
  int retpos;  
  for (int b=10; b<=11; b++) {       
    retpos = ics3.set_position(b, 7500);
    debugPrint(String(retpos) + "\r\n" );
    delay(1000);
 
    retpos = ics3.set_position(b, 8000);
    debugPrint(String(retpos) + "\r\n" );
    delay(1000);

    retpos = ics3.set_position_weakandkeep(b);
    debugPrint(String(retpos) + "\r\n" );
    delay(1000);
  }  
*/

  
/*
  //パラメータ　読み書き　テスト (EEPROM以外)
  int retval;
  bool test = false;
  for (int b=10; b<=11; b++) {    
    debugPrint("#######################################\r\n");
    retval = ics3.get_stretch(b);
    debugPrint("ID: " + String(b,DEC) + ", stretch: " + String(retval,DEC) + "\r\n");
    retval = ics3.get_speed(b);
    debugPrint("ID: " + String(b,DEC) + ", speed: " + String(retval,DEC) + "\r\n");
    retval = ics3.get_current(b);
    debugPrint("ID: " + String(b,DEC) + ", current: " + String(retval,DEC) + "\r\n");
    retval = ics3.get_temperature(b);
    debugPrint("ID: " + String(b,DEC) + ", temperature: " + String(retval,DEC) + "\r\n");
    delay(1000);
    if (test) { ics3.set_stretch(b, 100); }
    else { ics3.set_stretch(b, 99); }
    retval = ics3.get_stretch(b);
    debugPrint("ID: " + String(b,DEC) + ", stretch: " + String(retval,DEC) + "\r\n");
    test = !test;
    if (test) { delay(1000); }
    else { delay(10000); }
  }
*/


/*
  //EEPROMデータ読み取り
  EEPROMdata reted;
  int retcode = ics3.get_EEPROM(2, &reted);
  if (retcode != RETCODE_OK ) {
    debugPrint("getEEPROM modoriti error.\r\n");
    return ;
  }
  ics3.show_EEPROMdata(&reted);
  delay(2000);
*/



/*
  //EEPROM書き込み　テスト
  int retcode;
  EEPROMdata newed;  
  EEPROMdata reted;  
  
  newed.stretch = 100;
  newed.speed = 100;
  newed.punch = 0;
  newed.deadband = 2;
  newed.dumping = 25;
  newed.safetimer = 250;
  newed.flag_slave = 0;
  newed.flag_rotation = 0;
  newed.flag_pwminh = 1;
  newed.flag_free = 1;
  newed.flag_reverse = 0;
  newed.poslimithigh = 11500;
  newed.poslimitlow = 3500;
  newed.commspeed = 1250000;
  newed.temperaturelimit = 40;
  newed.currentlimit = 40;
  newed.response = 1;
  newed.offset = 0;
  newed.ID = 2;
  newed.charstretch1 = 60;
  newed.charstretch2 = 30;
  newed.charstretch3 = 90;
  
  ics3.show_EEPROMdata(&newed);

  retcode = ics3.set_EEPROM(2, &newed);  
  if (retcode != RETCODE_OK) {
    debugPrint("set_EEPROMdata error.\r\n");
    return ;
  } else {
    debugPrint("set_EEPROMdata OK!\r\n");
  }
  delay(1000);
  
  retcode = ics3.get_EEPROM(2, &reted);
  if (retcode != RETCODE_OK) {
    debugPrint("get_EEPROM error.\r\n");
    return ;
  } else {
    debugPrint("get_EEPROM OK!\r\n");
  }
  ics3.show_EEPROMdata(&reted);

  delay(3000);
*/



/*
  //ID存在テスト
  bool servo_table[3][ID_MAX+1];
  int baudratelist[3] = {1250000, 625000, 115200};
  //テーブルは[0]:1250000bps, [1]:625000, [2]:115200 とする
  //ID 0-31の計32個、存在をBOOLで保管 あればtrue

  debugPrint("##########################################\r\n");

  for (int a=0; a<=2; a++) {
    debugPrint("ID check: " + String(baudratelist[a]) + "bps.\r\n");
    ics3.change_baudrate(baudratelist[a]);
    
    for (int b=ID_MIN; b<=ID_MAX; b++) {
      servo_table[a][b] = ics3.IsServoAlive(b);    
      debugPrint(String(servo_table[a][b]) + "," );
    }
    debugPrint("\r\n");
  }
  ics3.change_baudrate(baudratelist[0]);
  delay(1000);
*/


/*
  //コマンド必要時間チェック用
  //transceive 内のデバッグ表示部をコメントアウト除去してから実行すると所要時間が測れる
  delay(10000);
  
  debugPrint("servo move\r\n" );
  for (int a=0; a<10; a++) {
    retval = set_position(1, 7500);
    delay(200);
    
    retval = set_position(1, 8000);
    delay(200);
  }
  delay(2000);

  debugPrint("servo weak\r\n" );
  for (int a=0; a<10; a++) { 
    retval = set_position(1, 7500);
    delay(200);

    retval = set_position(1, 8000);
    delay(200);

    retval = set_position_weak(1);
    delay(200);
  }  
  delay(2000);

  debugPrint("parameter read\r\n" );
  for (int a=0; a<10; a++) {
    retval = get_stretch(1);
    delay(200);
  }
  delay(2000);

  debugPrint("parameter write\r\n" );
  for (int a=0; a<10; a++) {
    set_stretch(1, 64);
    delay(200);
  }
  delay(2000);

  debugPrint("read EEPROM\r\n" );
  EEPROMdata tested;
  for (int a=0; a<10; a++) {
    get_EEPROM(1, &tested);
    delay(200);
  }
  delay(2000);  
  
  debugPrint("write EEPROM\r\n" );
  EEPROMdata newed;
  for (int a=0; a<10; a++) {   
    newed.offset = 0;
    retcode = set_EEPROM(1, &newed);
    delay(200);
  }
  delay(2000);


*/

    
}
