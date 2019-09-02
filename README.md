# STM32-Arduino-ICSservo
STM32 x Arduino にて近藤科学ICS 3.5 サーボを使用するライブラリです。
<br>
<br>近藤科学のICSサーボは最大1.25Mbpsと高い通信速度で処理を行えますが、STM32F103等は48～72MHzの高い処理能力を持っており、1.25Mbpsでの処理が可能です。
<br>当ライブラリではその基本機能を提供します。
<br>接続回路は、1線式UARTを想定しています。
<br>プリメイドAIハックで初めてロボットいじりの楽しさを知ったので、是非いろんな方に使ってほしいです。
<br>近藤科学公式のArduino ICSサーボライブラリとの違いは、2線式か1線式か、の利用回路の違いです。
<br>
<br>
# ●注意！
（<b>サーボ移動等の通常利用は全く問題ないと思いますが</b>、EEPROM書き換えを試す方はこちらをお読みください。）
<br>私の環境では全機能を利用できていますが、EEPROM書き替えについては書き込みデータ内容を誤るとサーボの基板の修理が必要になる場合があります。
<br>EEPROM書き込みコードに関しては、可及的にデータの整合性をチェックしたコードを意識しました。
<br>しかし利用にあたり、できましたらまずはEEPROM書き替え以外の安全な基本機能からご利用頂き（サーボ移動やパラメータ系、EEPROM読み出し）、必要であればその後サーボ一台でEEPROM書き込みをお試しください！　エラーがありましたら、エラーコードとともに情報をお教え頂ければ幸いです。
<br>EEPROM書き込み中にサーボ電源を切るのは危ないと思います。
<br><b>具体的には　set_EEPROM　関数の利用についてご注意ください、ということです。</b>
<br>心配な方は、set_EEPROM以外のみご利用いただき、パラメータ書き込みは公式のICSマネージャにて書き換えてください。
<br>
<br>
# ●機能
・<b>サーボ移動、脱力（現在位置確認も）</b>
<br>・<b>ID 読み書き</b>（通常コマンド、ホスト－サーボを１対１接続時）
<br>・<b>全パラメータ読み書き（EEPROM含む）</b>（ストレッチ、スピード、パンチ、デッドバンド、ダンピング、セイフタイマー、回転モード、PWM禁止、リバースモード、上下リミット、通信速度、温度制限、電流制限、レスポンス、オフセット、キャラスタリスティックチェンジのストレッチ1-3）</b>
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
コード loop 内処理サンプルをご覧ください。
<br>あえて１ファイルにクラスのコードをまとめてありますので、必要に応じ適宜別ファイルに移すなどしてください。
<code>
IcsCommunication ics2(Serial2);
void setup() {
  ics2.begin(SERIAL2_BAUDRATE, 50, true);   //初期化
}
void loop() {
    int retval = set_position(1, 7500);     //サーボ移動（ID, ポジション）
} 
</code>
<br>
<br>
# ●補足
秋月にたくさん売ってるNucleo ボードもSTM32シリーズが使われていますので、当ライブラリが使用できるかと思います。
<br>また、内部のtransceive 関数と通信速度等を書き替えれば他機種でも使えるかと思います。（他Arduino機種など）
<br>上位のICS 3.6規格には現在位置取得コマンドが存在しますが、それもtranscieve 関数を利用してもらえば、簡単に追加実装できると思います。
<br>
<br>
# ●作成者
devemin
<br>
<br>
# ●ライセンス
<br>MIT Liscence
<br>
<br>
# ●Thanks!
<br>@GOROman         ( https://twitter.com/GOROman )
<br>@izm             ( https://twitter.com/izm)
<br>@kazzlog         ( https://twitter.com/kazzlog )
<br>@Schwarz_Sardine ( https://twitter.com/Schwarz_Sardine )
<br>@witch_kazumin   ( https://twitter.com/witch_kazumin )
<br>and my twitter-follower...!
<br>
# ●注意事項
当ライブラリから発生する事象に対し、責任は全て利用者にあります。
<br>コードをよくご理解の上、ご利用下さい。
<br>
<br>
# ●更新履歴
ver 0.50:  publish
<br>
