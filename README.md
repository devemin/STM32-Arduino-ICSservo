# STM32-Arduino-ICSservo
STM32 x Arduino にて近藤科学ICS 3.5 サーボを使用するライブラリです。（プリメイドAI含む）
<br>
<br>接続回路は、1線式UARTを想定しています。
<br>プリメイドAIハックで初めてロボットいじりの楽しさを知ったので、是非いろんな方に使ってほしいです。
<br>何かありましたら、Issues(掲示板)の方でお教えいただけると幸いです。
<br>
<br>近藤科学公式のArduino ICSサーボライブラリとの違いは、2線式か1線式か、の利用回路の違いです。
<br>公式の回路は３ステートバッファを用意するものですが、1線式は回路がシンプルになるのがメリットです。
<br>
<br>
# ●機能
・<b>サーボ移動、脱力（現在位置確認も）</b>
<br>・<b>ID 読み書き</b>（通常コマンド、ホスト－サーボを１対１接続時）
<br>・<b>全パラメータ読み書き（EEPROM含む）</b>（ストレッチ、スピード、パンチ、デッドバンド、ダンピング、セイフタイマー、回転モード、PWM禁止、リバースモード、上下リミット、通信速度、温度制限、電流制限、レスポンス、オフセット、キャラスタリスティックチェンジのストレッチ1-3）</b>
<br>（<b>EEPROM書き込みを行う set_EEPROM 関数のみ、現在はベータ版としておきます。必要な方のみset_EEPROM関数をお使いください。詳しくは下の方に書きました。</b>）
<br>
<br>・<b>（独自）サーボ脱力後に同位置で即動作</b>（現在位置確認用として）
<br>・<b>（独自）ID読み書き</b>（EEPROM書き替えにより、複数接続時でも可能）
<br>・<b>（独自）ID指定によるサーボ存在確認</b>
<br>
<br>
# ●動作確認
プリメイドAI（STM32F102）、BluePill（同F103）
<br>
<br>
# ●利用する想定回路
プリメイドAIであれば、回路はそのままで利用できます。
<br>
<br>BluePill（STM32F103）等であれば、
<br>STM32F103 - レベルコンバータ（3.3V-5V） - ICSサーボ
<br>接続するpin番号 は、
<br>Serial2: PA2
<br>Serial3: PB10
<br>となっております。
<br>(Serial1: PA9)　当ライブラリではSerial1はデバッグ用として利用想定しています。Serial2-3を利用下さい。
<br>レベルコンバータは、私は秋月電子のこちらを利用しました。（3.3Vでも動いてはいましたが。）
<br>４ビット双方向ロジックレベル変換モジュール　ＢＳＳ１３８使用
<br>http://akizukidenshi.com/catalog/g/gK-13837/
<br>電源・GNDは別途用意してください。信号線（5V）、HV系(9-12V)、MV系（6-7.4V）
<br>私は電流制限抵抗・プルアップ抵抗も接続しました。
<br>
<br>
# ●利用方法
https://github.com/devemin/Pre-maiduino
<br>をご覧いただき、Arduino IDE にSTM32ボード情報をインストールしてください。
<br>プリメイドAIの場合は、上記に記載の通り、書き込み保護解除とともに、元ファーム消去が必要となります（元ファームのバックアップ不可）。
<br>STM32とPCの接続は、ST Linkケーブルを利用します。
<br>その後、当ライブラリのコードをArduino IDE にて開いてビルド、STM32に転送してもらえば利用できるかと思います。
<br>
<br>
# ●コード例
コード loop 内に処理サンプルを書いてありますので、ご覧ください。
<br>あえて１ファイルにクラスのコードをまとめてありますので、必要に応じ適宜別ファイルに移すなどしてください。
  
```cpp
IcsCommunication ics2(Serial2); //HV
IcsCommunication ics3(Serial3); //MV

void setup() {
  ics2.begin(SERIAL2_BAUDRATE, 50, true);   //初期化
  ics3.begin(SERIAL3_BAUDRATE, 50, true);
}
void loop() {
  int retval;
  retval = ics3.set_position(11, 7500);     //サーボ移動（ID, ポジション）
  delay(500);
  retval = ics3.set_position(11, 8000);
  delay(500);
} 
```


<br>
<br>
#●EEPROM書き込みを行う set_EEPROM 関数について
EEPROM書き込み処理は自分の個体では問題なく使えてますし、充分注意してコーディングしたつもりですが、万が一バグで書き込みデータが狂うと最悪サーボ修理となります。一応この関数はベータ版としておきます。set_EEPROM 関数以外は、そういうことは無いので安全に使えます。
<br>set_EEPROM 関数をご利用の方は、まず1か所（手先など）からお試し頂き、その後他の場所に使ってみてください。
<br>処理時間は1回 500msほどかかります。
<br>
<br>
#●補足
秋月にたくさん売ってるNucleo ボードもSTM32シリーズが使われていますので、当ライブラリが使用できるかと思います。
<br>また、内部のtransceive 関数と通信速度等を書き替えれば他機種でも使えるかと思います。（他Arduino機種など）
<br>上位のICS 3.6規格には現在位置取得コマンドが存在しますが、それもtranscieve 関数を利用してもらえば、簡単に追加実装できると思います。
<br>
<br>
#●作成者
devemin
<br>
<br>
#●ライセンス
<br>MIT Liscence
<br>
<br>
#●Thanks!
@GOROman         ( https://twitter.com/GOROman )
<br>@izm             ( https://twitter.com/izm)
<br>@kazzlog         ( https://twitter.com/kazzlog )
<br>@Schwarz_Sardine ( https://twitter.com/Schwarz_Sardine )
<br>@witch_kazumin   ( https://twitter.com/witch_kazumin )
<br>and my twitter-follower...!
<br>
#●注意事項
当ライブラリから発生する事象に対し、責任は全て利用者にあります。
<br>コードをよくご理解の上、ご利用下さい。
<br>
<br>
#●更新履歴
ver 0.50:  publish
<br>
