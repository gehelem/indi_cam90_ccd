unit cam90;

interface

uses Classes, SysUtils, MyD2XX, MMSystem, Windows, SyncObjs, ExtCtrls;

type
      {Class for reading thread}
      posl = class(TThread)
      private
      { Private declarations }
      protected
       procedure Execute; override;
      end;

const CameraWidth  = 3900;    //������ �����������
      CameraHeight = 2610;    //������ �����������
      FWidth = 3964;
      FHeight = 2720;
      CW2 = CameraWidth div 2;
      CH2 = CameraHeight div 2;
      portfirst = $11;        //�������������� �������� �� ������� ����� BDBUS

      spusb = 20000;          //bitbang velocity

      TemperatureOffset = 1280;
      MinErrTemp = -120.0;
      MaxErrTemp = 120.0;

{GLobal variables}
var   IsConnected : boolean = false;        //����������-����, ���������� ��������� ���������� � �������
      adress : integer;                     //��������� �������� ������ � �������� ������ FT2232HL
      mBin : integer;                       //�������,
      mImageReady : boolean = false;        //����������-����, ���������� ���������� � ���������� �����
      mCameraState : integer = 0;           //����������-��������� ������  0 - ready 1 - longexp 2 - read
      ExposureTimer : integer;              //������ ����������
      co: posl;                             //���������� ��� ������� ������ (������ �����������)
      bufim:array[0..CameraHeight*CameraWidth-1] of word;       //�������� ������-����������� ��� ��������
      bufi2:array[0..3*CameraHeight*CameraWidth-1] of word;     //�������� ������-����������� RGB
      mYn,mdeltY:integer;                   //������ ������ � ���������� �� �������
      //mXn,mdeltX:integer;                   //������ ������ � ���������� �� ��������
      zatv:byte;
      kolbyte:integer;
      eexp:integer;
      sm:integer;

      indval:integer;

     // ss:integer;
      siin:  array[0..3] of byte;
      siout: word;//array[0..3] of byte;

      //cached values
      sensorTempCache : Double;

procedure Spi_comm(comm:byte;param:word);      
function CameraConnect ()      : WordBool;
function CameraDisConnect ()   : WordBool;
function colomn_1()             : WordBool;
function Qbuf()                : integer;
function Rval()                : integer;
function CameraSetGain (val : integer) : WordBool;
function CameraSetOffset (val : integer) : WordBool;
function CameraSetColor (val : integer) : WordBool;
function CameraStartExposure (Bin,StartX,StartY,NumX,NumY : integer; Duration : double; light : WordBool) : WordBool;
function CameraStopExposure : WordBool;
function CameraSetTemp(temp:double): WordBool;
function CameraGetTemp ()      : Double;
function cameraGetPower ()     : word;
function CameraCoolingOn ()    : WordBool;
function CameraCoolingOff ()   : WordBool;

implementation

{ ��������� ��������� ������ � FT2232LH.
 ������ ������������ ����� �����:
  1. ������� ����������� ����� � ��������� ������� (����������� ������������������ ��������� �� ������� ����� BDBUS).
��� ���� ���������������� ��������� adress.
  2. ����� ���� ���� ������ ���������� �� ����� ��������: n:=Write_USB_Device_Buffer(FT_CAM8B,adress);
������������� ���������� FT2232HL ������ ��� �������� ��� ��� �������� �� ���� ���� BDBUS. �������� 1 ����� ��� ���� �������� 65 ��.
����� ��������� ��������� ������� n:=Write_USB_Device_Buffer(FT_CAM8B,adress) ������� �� ������������� ����������� � �� ��������������
����. ������� ����������� ������������������ ��������� ����� ��������� ���, � �� ���������� �� �������.
����� ���������� ����� �������� ��� ��������� (� ���� ��������� �� 24 �����!) ��� ����� ����� �������� ����� D2XX.pas, � ������ ��� MyD2XX.pas}

function Qbuf():integer;
begin
 Get_USB_Device_QueueStatus(FT_HANDLEA);
 result:=FT_Q_Bytes;
end;

function Rval():integer;
begin
 result:=indval;
end;

function colomn_1(): WordBool;
begin
Spi_comm($1a,0);
Result :=true;
end;

procedure sspi;
var
i,j:integer;
b:byte;
n:word;
begin
n:=100;
FillChar(FT_Out_Buffer,n,portfirst);
for j:=0 to 2 do
begin
b:=siin[j];
For i:= 0 to 7 do
begin
 inc(FT_Out_Buffer[2*i+1+16*j],$20);
 if (b and $80) = $80 then begin inc(FT_Out_Buffer[2*i+16*j],$80);inc(FT_Out_Buffer[2*i+1+16*j],$80);end;    //B3
 b:=b*2;
end;
end;
Write_USB_Device_Buffer(FT_HANDLEB,@FT_Out_Buffer,n);
end;

procedure sspo;
var i:integer;
b:word;
n:word;
begin
 n:=100;
 Read_USB_Device_Buffer(FT_HANDLEB,n);
    b:=0;
     for i:=0 to 15 do
      begin
       b:=b*2;
       if (FT_In_Buffer[i+1+8] and $40) <> 0 then inc(b);        //B4
      end;
      siout:=b;
end;

procedure Spi_comm(comm:byte;param:word);
begin
Purge_USB_Device_In(FT_HANDLEB);
Purge_USB_Device_Out(FT_HANDLEB);
siin[0]:=comm;
siin[1]:=hi(param);
siin[2]:=lo(param);
sspi;
sspo;
sleep(20);
end;

procedure ComRead;
begin
  co:=posl.Create(true);
  co.FreeOnTerminate:=true;
  co.Priority:=tpNormal;//Lower;//st;//r;//Normal;
  co.Resume;
end;

procedure posl.Execute;                                     //���������� ���� ������ ������� ����� ���� ADBUS
{ ������ �������������� ���������� ������ FT2232HL � �������� ������ �����������
  ��-�� ������������ AD9822 ��������� ������� ������� ����, ����� �������, � � delphi ��������.
  ���������� �����  ��� integer32, � �� word16 ��-�� ������������ ��� ����������� ��������� }
const
FWdiv2 = FWidth div 2;
var
x,y:integer;
i,j:integer;
dd:array[0..3] of integer;
//fff:file;
begin
 Read_USB_Device_Buffer(FT_HANDLEA,kolbyte);
 {assignfile(fff,'fff.bin');
 rewrite(fff,kolbyte);
 blockwrite(fff,FT_In_Buffer,1);
 closefile(fff); }
 fillchar(bufim,sizeof(bufim),0);
 if mBin = 0 then
 begin
 for x:= 0 to CW2-1 do
  begin
    for y:=0 to CH2-1 do
      begin
       i:= 2*y+32+(CW2-1-x+16)*FHeight;
       j:=i+FWdiv2*FHeight;
       dd[0]:=swap(FT_In_Buffer[0+i])-sm;    //gr1
       dd[1]:=swap(FT_In_Buffer[1+i]);    //red
       dd[2]:=swap(FT_In_Buffer[0+j])-sm;    //gr2
       dd[3]:=swap(FT_In_Buffer[1+j]);    //blue

    bufim[2*x+0+(2*y+0)*CameraWidth]:=dd[2];
    bufim[2*x+1+(2*y+1)*CameraWidth]:=dd[0];
    bufim[2*(CW2-x)-1+(2*y+0)*CameraWidth]:=dd[3];
    bufim[2*(CW2-x)-2+(2*y+1)*CameraWidth]:=dd[1];
      end;
  end;
  end        else
  begin
  for x:= 0 to CW2-1 do
  begin
     for y:=0 to CH2-1 do
      begin
       i:= 2*y+32+(CW2-1-x+16)*FHeight;
       dd[0]:=swap(FT_In_Buffer[0+i]);
       dd[1]:=swap(FT_In_Buffer[1+i]);

       dd[0]:=dd[0]+bufim[2*x+0+(2*y+0)*CameraWidth];
       if dd[0] > 65535 then dd[0]:=65535;
       bufim[2*x+0+(2*y+0)*CameraWidth]:=dd[0];
       bufim[2*x+0+(2*y+1)*CameraWidth]:=dd[0];
       bufim[2*x+1+(2*y+0)*CameraWidth]:=dd[0];
       bufim[2*x+1+(2*y+1)*CameraWidth]:=dd[0];
       dd[1]:=dd[1]+bufim[2*(CW2-x)-1+(2*y+0)*CameraWidth];
       if dd[1] > 65535 then dd[1]:=65535;
       bufim[2*(CW2-x)-1+(2*y+0)*CameraWidth]:=dd[1];
       bufim[2*(CW2-x)-1+(2*y+1)*CameraWidth]:=dd[1];
       bufim[2*(CW2-x)-2+(2*y+0)*CameraWidth]:=dd[1];
       bufim[2*(CW2-x)-2+(2*y+1)*CameraWidth]:=dd[1];
      end;
  end;
  end;
  mCameraState:=0;
  mImageReady := true;
end;

{���������� ��������� ������ �������� ��� �������� � ���������� ����� val �� ������ adr � ���������� AD9822.
 �������� ���� � ���������������� ����.}
procedure AD9822(adr:byte;val:word);
const
kol = 64;
var
dan:array[0..kol-1] of byte;
i:integer;
begin
 fillchar(dan,kol,portfirst);                                   //����������� ������ �������������� ��������� �� ������� ����� BDBUS
 for i:=1 to 32 do dan[i]:=dan[i] and $fe;
 for i:=0 to 15 do dan[2*i+2]:=dan[2*i+2] + 2;
 if (adr and 4) = 4 then begin dan[3]:=dan[3]+4;dan[4]:=dan[4]+4;end;
 if (adr and 2) = 2 then begin dan[5]:=dan[5]+4;dan[6]:=dan[6]+4;end;
 if (adr and 1) = 1 then begin dan[7]:=dan[7]+4;dan[8]:=dan[8]+4;end;

 if (val and 256) = 256 then begin dan[15]:=dan[15]+4;dan[16]:=dan[16]+4;end;
 if (val and 128) = 128 then begin dan[17]:=dan[17]+4;dan[18]:=dan[18]+4;end;
 if (val and 64) = 64 then begin dan[19]:=dan[19]+4;dan[20]:=dan[20]+4;end;
 if (val and 32) = 32 then begin dan[21]:=dan[21]+4;dan[22]:=dan[22]+4;end;
 if (val and 16) = 16 then begin dan[23]:=dan[23]+4;dan[24]:=dan[24]+4;end;
 if (val and 8) = 8 then begin dan[25]:=dan[25]+4;dan[26]:=dan[26]+4;end;
 if (val and 4) = 4 then begin dan[27]:=dan[27]+4;dan[28]:=dan[28]+4;end;
 if (val and 2) = 2 then begin dan[29]:=dan[29]+4;dan[30]:=dan[30]+4;end;
 if (val and 1) = 1 then begin dan[31]:=dan[31]+4;dan[32]:=dan[32]+4;end;

 Write_USB_Device_Buffer(FT_HANDLEB,@dan, kol);
end;

{������������ 2 ������:
 1.������� ��� �������.
 2.�/� � �������� 2*2.
 ������������ ������� ICX453 �������� ��, ��� �������������� ������� ����� ��������� ������� �
 ��� ����� ���� ������������� ������ � �������������� ������� "������" ����� ���� �����,
 ������� ���������� ����� ��� ���� ���� �������� ����������.
}

{���������� ��������� ������ �������� � ���������� ���� �������� ������ ����� � 1 ������}
procedure readframe;
begin
 mCameraState := 2;
 mImageReady:=false;
 //Purge_USB_Device_IN(FT_HANDLEA);
 Purge_USB_Device_OUT(FT_HANDLEA);
 comread;
 Spi_comm($1b,0);//$ffff);
end;

{���������� ��������� ������ �������� � ���������� ���� �������� ������ ����� � 1 ������}
procedure readframe2;
begin
 mCameraState := 2;
 mImageReady:=false;
 //Purge_USB_Device_IN(FT_HANDLEA);
 Purge_USB_Device_OUT(FT_HANDLEA);
 comread;
 Spi_comm($10,0);//$ffff);
end;

{Set camera gain, return bool result}
function CameraSetGain (val : integer) : WordBool;// stdcall; export;
begin
 AD9822(3,val);           //gain AD9822 green
 AD9822(2,val);           //gain AD9822 red
 Result :=true;
end;

{Set camera offset, return bool result}
function CameraSetOffset (val : integer) : WordBool;
var x : integer;
begin
 x:=abs(2*val);
 if val < 0 then x:=x+256;
 AD9822(6,x);                       //offset AD9822 green
 AD9822(5,x);                       //offset AD9822 red
 Result :=true;
end;

{Set camera offset, return bool result}
function CameraSetColor (val : integer) : WordBool;
begin
 sm:=val;
 Result :=true;
end;

{Connect camera, return bool result}
{����� ������������ ��������� � ������������� AD9822}
function CameraConnect () : WordBool;
var  FT_flag, FT_OP_flag : boolean;
I : Integer;
ress:byte;
begin
 FT_flag:=false;
 FT_Enable_Error_Report:=false;
 sensorTempCache := 0;
 GetFTDeviceCount;
 I := FT_Device_Count-1;
 while I >= 0 do
  begin
   GetFTDeviceSerialNo(I);
   if pos('CAM90',FT_Device_String) <> 0 then FT_flag:=true;    //���� ��������� cam90 - ����������
   GetFTDeviceDescription(I);
   Dec(I);
  end;
  FT_OP_flag:=true;
  if FT_flag then
   begin
    if Open_USB_Device_By_Serial_Number(FT_HANDLEA,'CAM90A') <> FT_OK then FT_OP_flag := false;
    if Open_USB_Device_By_Serial_Number(FT_HANDLEB,'CAM90B')  <> FT_OK then FT_OP_flag := false;

    if Set_USB_Device_BitMode(FT_HANDLEB,$bf, $4)  <> FT_OK then FT_OP_flag := false;
    FT_Current_Baud:=spusb;                         // BitMode for B-canal volocity = spusb
    Set_USB_Device_BaudRate(FT_HANDLEB);

    Set_USB_Device_LatencyTimer(FT_HANDLEB,1);       //������������ ��������������
    Set_USB_Device_LatencyTimer(FT_HANDLEA,1);
    Set_USB_Device_TimeOuts(FT_HANDLEA,8000,100);
    Set_USB_Device_TimeOuts(FT_HANDLEB,100,100);
    Set_USB_Parameters(FT_HANDLEA,65536,0);

    Purge_USB_Device_IN(FT_HANDLEA);
    Purge_USB_Device_OUT(FT_HANDLEA);
    Purge_USB_Device_IN(FT_HANDLEB);
    Purge_USB_Device_OUT(FT_HANDLEB);

    adress:=0;

    AD9822(0,$d8);//$58);             //����� AD9822 - ����� G,4 ������ ���������, CDS �����
    AD9822(1,$e0);//$e0 //$a0);

    CameraSetGain(0);         //�������� ��������������� �����. ��� �� ������������� ���
    CameraSetOffset(-6);
    
    {reset AVR}
    ress:=portfirst-$10;
    Write_USB_Device_Buffer(FT_HANDLEB,@ress, 1);
    sleep(10);
    ress:=portfirst;
    Write_USB_Device_Buffer(FT_HANDLEB,@ress, 1);

    //send init command
   // Spi_comm($db,0);
    sleep(100);

    Purge_USB_Device_IN(FT_HANDLEA); //������ 2 �����, ��������� ����� reset
    mBin:=0;

    mCameraState:=0;
   end;
 IsConnected := FT_flag and FT_OP_flag;
 Result := FT_flag and FT_OP_flag;
end;


{Disconnect camera, return bool result}
function CameraDisConnect (): WordBool;
var FT_OP_flag : boolean;
begin
 FT_OP_flag := true;
 if Close_USB_Device(FT_HANDLEA) <> FT_OK then FT_OP_flag := false;   //�������� ���������
 if Close_USB_Device(FT_HANDLEB) <> FT_OK then FT_OP_flag := false;
 IsConnected := not FT_OP_flag;
 Result:= FT_OP_flag;
end;

procedure ExposureTimerTick(TimerID, Msg: Uint; dwUser, dw1, dw2: DWORD); stdcall;
begin
 dec(indval);
 if indval <= 0 then
 begin
  KillTimer (0,ExposureTimer);
  Spi_comm($cb,0); //clear frame
  sleep(180);                           // for time of clear frame
  readframe2;
 end;
end;

{Check camera connection, return bool result}
function CameraIsConnected () : WordBool;// stdcall; export;
begin
  Result := IsConnected;
end;

function CameraStartExposure (Bin,StartX,StartY,NumX,NumY : integer; Duration : double; light : WordBool) : WordBool;// stdcall; export;
var
//d0,d1:word;
expoz:integer;
begin

 mYn:=StartY div 2;
 Spi_comm($4b,mYn);
 mdeltY:=NumY div 2;
 Spi_comm($5b,mdeltY);

 mBin := Bin;
 if bin = 1 then
 begin
 kolbyte:=10782080;//3964*2720
 Spi_comm($8b,1);  //bining
 mBin:=1;
 end            else
 begin
 kolbyte:=21564160;//3964*2720*2 
 Spi_comm($8b,0);  //no bining
 mBin:=0;
 end;

 expoz:=round(Duration*1000);
 Spi_comm($6b,expoz);

 mImageReady := false;
 //camera exposing
 mCameraState := 1;
 if Duration > 2.0 then
 begin
  eexp:=round(1000*Duration);
  indval:=eexp div 1000;
  ExposureTimer := SetTimer(0,0,1000,@ExposureTimerTick);
  Spi_comm($2b,0); //shift3
  sleep(100);
  Spi_comm($3b,0); //off 15v
 end                   else
 begin
  eexp:=0;
  readframe;
 end;
 Result := true;
end;

function CameraStopExposure : WordBool;// stdcall; export;
begin
 indval:=0;
 KillTimer (0,ExposureTimer);
 if mCamerastate = 1 then readframe2;
 Result := true;
end;

function cameraGetTemp (): double;
var temp : double;
begin
    Spi_comm($bf,0);
    temp := (siout - TemperatureOffset) / 10.0;
    if ((temp > MaxErrTemp) or (temp < MinErrTemp)) then
    begin
        temp := sensorTempCache;
    end;
    sensorTempCache := temp;
    Result := temp;
end;

function cameraGetPower (): word;
begin
 Spi_comm($bc,0);
 Result :=siout;
end;

function cameraSetTemp(temp : double): WordBool;
var d0:word;
begin
    d0 := TemperatureOffset + round(temp*10);
    Spi_comm($ab,d0);
    Result := true;
end;

function CameraCoolingOn (): WordBool;
begin
 Spi_comm($9b,1);
 Result := true;
end;

function CameraCoolingOff (): WordBool;
begin
 Spi_comm($9b,0);
 Result := true;
end;

end.
