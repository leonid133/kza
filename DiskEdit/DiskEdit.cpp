//---------------------------------------------------------------------------
#include <iostream>
#include <windows.h>
#include <fstream.h>

#include <vcl.h>
#pragma hdrstop

#include "DiskEdit.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
//---------------------------------------------------------------------------
   //Disk
   /*
    typedef struct _DISK_GEOMETRY
    {
        LARGE_INTEGER Cylinders; // Количество цилиндров
        DWORD MediaType;
        DWORD TracksPerCylinder; // Количество дорожек на цилиндр
        DWORD SectorsPerTrack;  // Количество секторов на дорожку
        DWORD BytesPerSector;
    }   DISK_GEOMETRY;         */

    DISK_GEOMETRY diskGeometry;

    DWORD bytesReturned=0;

    BYTE *buffer=NULL;

    DWORD bufferSize;

    BOOL result=0;
//---------------------------------------------------------------------------
TForm1 *Form1;
//---------------------------------------------------------------------------
__fastcall TForm1::TForm1(TComponent* Owner)
   : TForm(Owner)
{
//   SaveDialog1->DefaultExt = "*.av22";
//   SaveDialog1->Filter = "AV22 КЗА файлы (*.av22)";
}
//-----------------------------------------------------------------------------
void TForm1::SaveSektorToFile(String FileName)
{

}

void  TForm1::DriveRead(void)
{
    String DriveOpen="\\\\.\\"+DriveComboBox1->Items->Strings[DriveComboBox1->ItemIndex];
    DriveOpen.Insert('\0',7);
    wchar_t wstr[10] = {'\0'};
    swprintf(wstr, L"%S", DriveOpen.c_str());
    HANDLE hFile=CreateFileW(wstr ,GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,0,OPEN_EXISTING,0,0);
    if (hFile==INVALID_HANDLE_VALUE)
    {
       int err = GetLastError();
       ShowMessage("Ошибка #" + IntToStr(err) + ": Не удалось открыть диск!");
       CloseHandle(hFile);
       return;
    }

    DeviceIoControl(hFile,IOCTL_DISK_GET_DRIVE_GEOMETRY,NULL,0,&diskGeometry,sizeof (DISK_GEOMETRY),&bytesReturned,(LPOVERLAPPED)NULL);

    StringGrid1->RowCount=diskGeometry.BytesPerSector/16+1;

    bufferSize=diskGeometry.BytesPerSector;

    buffer = new BYTE[bufferSize];
    SetFilePointer(hFile,StrToInt(Edit1->Text)*bufferSize,NULL,FILE_BEGIN);
    result = ReadFile(hFile, buffer, bufferSize, &bytesReturned, 0);
    if (!result)
    {
       int err = GetLastError();
       ShowMessage("Ошибка #" + IntToStr(err));
       delete[] buffer;
       CloseHandle(hFile);
       return;
    }
    cout << buffer;
    //system("pause");
    String txt;
    StringGrid1->RowCount = (bufferSize/(int)(StringGrid1->ColCount-1))+1;
    int d_i=0;
    for (int i = 1; i < (int)StringGrid1->RowCount; i++)
       for (int j = 1; j < (int)StringGrid1->ColCount; j++){
            txt.sprintf("%x\n", j-1); StringGrid1->Cells[j][0] = txt;
            txt.sprintf("%x\n", (i-1)*(int)(StringGrid1->ColCount-1) + StrToInt(Edit1->Text)*bufferSize); StringGrid1->Cells[0][i] = txt;
            txt.sprintf("%x\n", buffer[((int)StringGrid1->ColCount-1)*(i-1)+(j-1)]); StringGrid1->Cells[j][i] = txt;
            d_i++;
            DataSektor[d_i] = buffer[((int)StringGrid1->ColCount-1)*(i-1)+(j-1)];
       }
    Label4->Caption = IntToStr(diskGeometry.MediaType);
    Label2->Caption = IntToStr(diskGeometry.TracksPerCylinder);
    Label6->Caption = IntToStr(diskGeometry.SectorsPerTrack);
    Label8->Caption = IntToStr(diskGeometry.BytesPerSector);
    Label14->Caption = IntToStr(diskGeometry.Cylinders.QuadPart);
    CloseHandle(hFile);
    delete[] buffer;
}
//---------------------------------------------------------------------------
  

void __fastcall TForm1::FormResize(TObject *Sender)
{
     StringGrid1->Width = Form1->Width-430;
     StringGrid1->Height = Form1->Height-100;
     Panel1->Left = Form1->Width - 420;
     //Panel2->Left = Form1->Width - 220;
     Panel3->Left = Form1->Width - 420;
     Image1->Left = Form1->Width - 220;
     Memo1->Left = Form1->Width - 420;
     /*Panel3->Width =  Form1->Width-230;
     Panel3->Height =  Form1->Height-90;    */
}
//---------------------------------------------------------------------------
void __fastcall TForm1::Button1Click(TObject *Sender)
{
//   Panel3->Visible = false;
  /*
   Button1->Visible = true;
   StringGrid1->Visible = true;
   Edit1->Visible = true;
   UpDown1->Visible = true;
   Panel2->Visible =true;
   Panel3->Visible = true;*/
   DriveRead();
}
//---------------------------------------------------------------------------
void __fastcall TForm1::Button2Click(TObject *Sender)
{
   Button1->Enabled = false;
   Button2->Enabled = false;
   Button3->Enabled = false;
   Button4->Enabled = false;

   SaveDialog1->Execute();
   int a,b;
   a = StrToInt(Edit2->Text);
   b = StrToInt(Edit3->Text);
   if(a>b)
   {
      int tmp;
      tmp = a;
      a=b;
      b=tmp;
   }
   fstream file;
   file.open(SaveDialog1->FileName.c_str(), ios:: out | ios::binary);
   if (!file)
   {
      String txt = "Запись не удалась сюда: ";
      txt += SaveDialog1->FileName.c_str();
      Application->MessageBoxA(txt.c_str(),"Опаньки :(",0);
      return;
   }
   else
   {
      if(a>b)
      {
        for(int i=a; i<=1945600; i++)
        {
            Edit1->Text = i;
            DriveRead();
            file.write(buffer, bufferSize);
            ProgressBar1->Position = 100*(i - a)/(1945600-a);
        }
        for(int i=7; i<=b; i++)
        {
            Edit1->Text = i;
            DriveRead();
            file.write(buffer, bufferSize);
            ProgressBar1->Position = 100*(i - 7)/(b-7);
        }
      }
      else
      {
         for(int i=a; i<=b; i++)
         {
            Edit1->Text = i;
            DriveRead();
            file.write(buffer, bufferSize);
            ProgressBar1->Position = 100*(i - a)/(b-a);
         }
      }
      String txt = "Запись удалась сюда: ";
      txt += SaveDialog1->FileName.c_str();
      Application->MessageBoxA(txt.c_str(),"Внимание",0);
   }
   file.close();
   Button1->Enabled = true;
   Button2->Enabled = true;
   Button3->Enabled = true;
   Button4->Enabled = true;

}
//---------------------------------------------------------------------------

void __fastcall TForm1::Edit1KeyPress(TObject *Sender, char &Key)
{
   Set <char, '0', '9'> Dig;
	Dig << '0' << '1' << '2' << '3' << '4' << '5' << '6' <<'7'<<'8'<< '9';

    if(Key==13 || Key==32)
    {
       Key=0;
       DriveRead();
    }
    else if(Key == 8)
    {
    }
    else if ( !Dig.Contains(Key) )
    {
	    Key = 0;
	    Beep();
    }
}
//---------------------------------------------------------------------------

void __fastcall TForm1::Button3Click(TObject *Sender)
{
    String DriveOpen="\\\\.\\"+DriveComboBox1->Items->Strings[DriveComboBox1->ItemIndex];
    DriveOpen.Insert('\0',7);
    wchar_t wstr[10] = {'\0'};
    swprintf(wstr, L"%S", DriveOpen.c_str());
    HANDLE hFile=CreateFileW(wstr ,GENERIC_WRITE, FILE_SHARE_WRITE,0,OPEN_EXISTING,0,0);
    if (hFile==INVALID_HANDLE_VALUE)
    {
       int err = GetLastError();
       ShowMessage("Ошибка #" + IntToStr(err) + ": Не удалось открыть диск!");
       CloseHandle(hFile);
       return;
    }


    buffer = new BYTE[bufferSize];
    SetFilePointer(hFile,StrToInt(Edit1->Text)*bufferSize,NULL,FILE_BEGIN);
    int d_i=0;
    for (int i = 1; i < (int)StringGrid1->RowCount; i++)
       for (int j = 1; j < (int)StringGrid1->ColCount; j++){
            d_i++;
            buffer[((int)StringGrid1->ColCount-1)*(i-1)+(j-1)] = DataSektor[d_i];
       }
    if(!WriteFile(hFile, buffer, bufferSize, &bytesReturned, 0))
    {
      int err = GetLastError();
      ShowMessage("Ошибка #" + IntToStr(err) + ": Не удалось записать на диск!");
       CloseHandle(hFile);
       return;
    }

    //cout << buffer;
    //system("pause");

    CloseHandle(hFile);
    delete[] buffer;
}
//---------------------------------------------------------------------------



void __fastcall TForm1::StringGrid1SetEditText(TObject *Sender, int ACol,
      int ARow, const AnsiString Value)
{
   StringGrid1->Cells[ACol][ARow] = Value;
}
//---------------------------------------------------------------------------



void __fastcall TForm1::Button4Click(TObject *Sender)
{
   Edit1->Text=8;
   Button1Click(Sender);
   Panel3->Visible = true;
   int nzap, god=1970, mesyac=0, day=0, hh=0, mm=0, ss=0;
   long time_t=0, addr=0;
   String txt, txt2;
   Memo2->Text = "";
   ListView1->Items->Clear();
   for(int slipbyte=1; slipbyte<45; slipbyte++)
   {
      BYTE CRC;
      CRC = DataSektor[slipbyte+1];
		for(int i = (slipbyte+2); i < (slipbyte*11+12); i++)
		{
			CRC = CRC ^ DataSektor[i];
		}

      if(CRC!=0)continue; 

      nzap =  DataSektor[slipbyte*11+2];
  		nzap |= (long)(DataSektor[slipbyte*11+1]<<8);
      if(nzap==0)continue;
      addr =  DataSektor[slipbyte*11+6];
      addr |= (long)(DataSektor[slipbyte*11+5]<<8);
      addr |= (long)(DataSektor[slipbyte*11+4]<<16);
		addr |= (long)(DataSektor[slipbyte*11+3]<<24);

      time_t = DataSektor[slipbyte*11+10];
      time_t |= (long)(DataSektor[slipbyte*11+9]<<8);
      time_t |= (long)(DataSektor[slipbyte*11+8]<<16);
		time_t |= (long)(DataSektor[slipbyte*11+7]<<24);

      txt = nzap;
      txt += ", ";
      //txt2.sprintf("%x\n", addr);
      addr /= bufferSize;
      txt += addr;
      txt += ", ";
      god=1970;
      while(time_t>31556926)
      {
         time_t-=31556926;
         god++;
      }
      mesyac =0;
      while(time_t>2629743)
      {
         time_t-=2629743;
         mesyac++;
      }
      day=0;
      while(time_t>86400)
      {
         time_t-=86400;
         day++;
      }
      hh=0;
      while(time_t>3600)
      {
         time_t-=3600;
         hh++;
      }
      mm=0;
      while(time_t>60)
      {
         time_t-=60;
         mm++;
      }
      ss=0;
      ss=time_t;
      txt += god;
      txt += "/";
      txt += mesyac;
      txt += "/";
      txt += day;
      txt += "/";
      txt += hh;
      txt += ":";
      txt += mm;
      txt += ":";
      txt += ss;
      ListView1->AddItem(slipbyte,Sender);
     // Memo1->Lines->Add(txt);
   }
}
//---------------------------------------------------------------------------

void __fastcall TForm1::ListView1SelectItem(TObject *Sender,
      TListItem *Item, bool Selected)
{
   Edit1->Text=8;
   Button1Click(Sender);
   Panel3->Visible = true;
   int nzap, god=1970, mesyac=0, day=0, hh=0, mm=0, ss=0;
   long time_t=0, addr=0;
   String txt, txt2;
   Memo2->Text = "";
   if(Selected)
   {
      int slipbyte;
      slipbyte=StrToInt(ListView1->Items->Item[Item->Index]->Caption);
      nzap =  DataSektor[slipbyte*11+2];
  		nzap |= (long)(DataSektor[slipbyte*11+1]<<8);
      addr =  DataSektor[slipbyte*11+6];
      addr |= (long)(DataSektor[slipbyte*11+5]<<8);
      addr |= (long)(DataSektor[slipbyte*11+4]<<16);
		addr |= (long)(DataSektor[slipbyte*11+3]<<24);

      time_t = DataSektor[slipbyte*11+10];
      time_t |= (long)(DataSektor[slipbyte*11+9]<<8);
      time_t |= (long)(DataSektor[slipbyte*11+8]<<16);
		time_t |= (long)(DataSektor[slipbyte*11+7]<<24);

      txt = nzap;
      txt += ", ";
      //txt2.sprintf("%x\n", addr);
      addr /= bufferSize;
      txt += addr;
            txt += ", ";
      god=1970;
      while(time_t>31556926)
      {
         time_t-=31556926;
         god++;
      }
      mesyac =0;
      while(time_t>2629743)
      {
         time_t-=2629743;
         mesyac++;
      }
      day=0;
      while(time_t>86400)
      {
         time_t-=86400;
         day++;
      }
      hh=0;
      while(time_t>3600)
      {
         time_t-=3600;
         hh++;
      }
      mm=0;
      while(time_t>60)
      {
         time_t-=60;
         mm++;
      }
      ss=0;
      ss=time_t;
      txt += god;
      txt += "/";
      txt += mesyac;
      txt += "/";
      txt += day;
      txt += "/";
      txt += hh;
      txt += ":";
      txt += mm;
      txt += ":";
      txt += ss;
      Memo2->Lines->Add(txt);
      
      Edit2->Text = addr;
      slipbyte++;
      if(slipbyte>45)slipbyte=0;
      addr =  DataSektor[slipbyte*11+6];
      addr |= (long)(DataSektor[slipbyte*11+5]<<8);
      addr |= (long)(DataSektor[slipbyte*11+4]<<16);
		addr |= (long)(DataSektor[slipbyte*11+3]<<24);
      addr /= bufferSize;

      time_t = DataSektor[slipbyte*11+10];
      time_t |= (long)(DataSektor[slipbyte*11+9]<<8);
      time_t |= (long)(DataSektor[slipbyte*11+8]<<16);
		time_t |= (long)(DataSektor[slipbyte*11+7]<<24);
      if(time_t > 1353038382)
      {
         Edit3->Text = addr;
      }
      else
      {
         addr = StrToInt(Edit2->Text)*bufferSize;
         while(true)
         {
            Edit1->Text=addr/bufferSize;
            Button1Click(Sender);
            nzap =  DataSektor[4];
  		      nzap |= (long)(DataSektor[3]<<8);
            if((DataSektor[0]==0x65 && DataSektor[1]==0x6f && DataSektor[2] == 0x66)||StrToInt(ListView1->Items->Item[Item->Index]->Caption)!=nzap)
            {
               Edit3->Text = addr/bufferSize;
               break;
            }
            addr =  DataSektor[8];
            addr |= (long)(DataSektor[7]<<8);
            addr |= (long)(DataSektor[6]<<16);
		      addr |= (long)(DataSektor[5]<<24);
         }
      }
   }
   Panel3->Visible = true;
}
//---------------------------------------------------------------------------


