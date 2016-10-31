#pragma warning(disable:4996)
#include <Windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <GL/glut.h>
#include <iostream>
#include <string>
using namespace std;

//Serial通信
HANDLE arduino;
bool Ret;
char data;
DWORD dwSendSize;
char In[20];
DWORD lRead;

//Arduinoから送られるデータの入れる変数
static int roll = 0.0f;
static int pitch = 0.0f;
static int yaw = 0.0f;
int go = 0;
// シリアル通信でroll,pitch,yawを取得する関数 
void connectSerial(){
	//6.送信
	data = 'n';
	//printf("%c\n", data);

	if (go == 0){
	Sleep(3000);
	go++;
	}

	Ret = WriteFile(arduino, &data, sizeof(data), &dwSendSize, NULL);
	if (!Ret){
		printf("SEND FAILED\n");
		CloseHandle(arduino);
		system("PAUSE");
		exit(0);
	}


	//7.受信
	Ret = ReadFile(   // データの受信
		arduino,   // 　通信デバイスのハンドル：　CreateFile()で取得したハンドルを指定
		In,       // 受信バッファーのポインタを指定：　受信データがここに格納されます。
		20,	//　受信するバイト数を指定：　ここで指定するバイト数を受信するかまたはタイムアウト時間がくるまで
		// ReadFile()関数は（　getc()のように　）待ちます
		&lRead,  //  実際に受信したバイト数（DWORD)が格納されるポインタを指定
		NULL   // 通信とは関係ない引数なのでNULLを指定
		);

	if (Ret == FALSE)     //失敗した場合
	{
		printf("ReadFile failed. \n");
		CloseHandle(arduino);
		exit(0);
	}

	char* str_roll = strtok(In, ",");
	char* str_pitch = strtok(NULL, ",");
	char* str_yaw = strtok(NULL, ",");

#define smooth_value 4	//平滑化量
	static int smooth_angle[smooth_value][3];
	static int count = 0;
	smooth_angle[count%smooth_value][0] = atoi(str_roll);
	smooth_angle[count%smooth_value][1] = atoi(str_pitch);
	smooth_angle[count%smooth_value][2] = atoi(str_yaw);
	count++;
	for (int i = 0; i < smooth_value; i++){
		roll += smooth_angle[i][0];
		pitch += smooth_angle[i][1];
		yaw += smooth_angle[i][2];
	}
	roll = (floor)((float)roll / smooth_value);
	pitch = (floor)((float)pitch / smooth_value);
	yaw = (floor)((float)yaw / smooth_value);

	cout << "data : " << roll << "," << pitch << "," << yaw << endl << endl;
}

/*
* 直方体を描く
*/
static void myBox(double x, double y, double z)
{
	GLdouble vertex[][3] = {
		{ -x, -y, -z },
		{ x, -y, -z },
		{ x, y, -z },
		{ -x, y, -z },
		{ -x, -y, z },
		{ x, -y, z },
		{ x, y, z },
		{ -x, y, z }
	};

	const static int face[][4] = {
		{ 0, 1, 2, 3 },
		{ 1, 5, 6, 2 },
		{ 5, 4, 7, 6 },
		{ 4, 0, 3, 7 },
		{ 4, 5, 1, 0 },
		{ 3, 2, 6, 7 }
	};

	const static GLdouble normal[][3] = {
		{ 0.0, 0.0, -1.0 },
		{ 1.0, 0.0, 0.0 },
		{ 0.0, 0.0, 1.0 },
		{ -1.0, 0.0, 0.0 },
		{ 0.0, -1.0, 0.0 },
		{ 0.0, 1.0, 0.0 }
	};

	const static GLfloat red[] = { 0.8, 0.2, 0.2, 1.0 };

	int i, j;

	/* 材質を設定する */
	glMaterialfv(GL_FRONT, GL_DIFFUSE, red);

	glBegin(GL_QUADS);
	for (j = 0; j < 6; ++j) {
		glNormal3dv(normal[j]);
		for (i = 4; --i >= 0;) {
			glVertex3dv(vertex[face[j][i]]);
		}
	}
	glEnd();
}

/*
* 円柱を描く
*/
static void myCylinder(double radius, double height, int sides)
{
	const static GLfloat yellow[] = { 0.8, 0.8, 0.2, 1.0 };
	double step = 6.28318530717958647692 / (double)sides;
	int i = 0;

	/* 材質を設定する */
	glMaterialfv(GL_FRONT, GL_DIFFUSE, yellow);

	/* 上面 */
	glNormal3d(0.0, 1.0, 0.0);
	glBegin(GL_TRIANGLE_FAN);
	while (i < sides) {
		double t = step * (double)i++;
		glVertex3d(radius * sin(t), height, radius * cos(t));
	}
	glEnd();

	/* 底面 */
	glNormal3d(0.0, -1.0, 0.0);
	glBegin(GL_TRIANGLE_FAN);
	while (--i >= 0) {
		double t = step * (double)i;
		glVertex3d(radius * sin(t), -height, radius * cos(t));
	}
	glEnd();

	/* 側面 */
	glBegin(GL_QUAD_STRIP);
	while (i <= sides) {
		double t = step * (double)i++;
		double x = sin(t);
		double z = cos(t);

		glNormal3d(x, 0.0, z);
		glVertex3f(radius * x, height, radius * z);
		glVertex3f(radius * x, -height, radius * z);
	}
	glEnd();
}

/*
* 地面を描く
*/
static void myGround(double height)
{
	const static GLfloat ground[][4] = {
		{ 0.6, 0.6, 0.6, 1.0 },
		{ 0.3, 0.3, 0.3, 1.0 }
	};

	int i, j;

	glBegin(GL_QUADS);
	glNormal3d(0.0, 1.0, 0.0);
	for (j = -5; j < 5; ++j) {
		for (i = -5; i < 5; ++i) {
			glMaterialfv(GL_FRONT, GL_DIFFUSE, ground[(i + j) & 1]);
			glVertex3d((GLdouble)i, height, (GLdouble)j);
			glVertex3d((GLdouble)i, height, (GLdouble)(j + 1));
			glVertex3d((GLdouble)(i + 1), height, (GLdouble)(j + 1));
			glVertex3d((GLdouble)(i + 1), height, (GLdouble)j);
		}
	}
	glEnd();
}

/*
* 画面表示
*/
static void display(void)
{
	//シリアル通信開始
	connectSerial();

	const static GLfloat blue[] = { 0.2, 0.2, 0.8, 1.0 };     /* 球の色 */
	const static GLfloat lightpos[] = { 3.0, 4.0, 5.0, 1.0 }; /* 光源の位置 */

	/* 画面クリア */
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/* モデルビュー変換行列の初期化 */
	glLoadIdentity();

	/* 光源の位置を設定 */
	glLightfv(GL_LIGHT0, GL_POSITION, lightpos);

	/* 視点の移動（シーンの方を奥に移す）*/
#if 1	//横から
	glTranslated(0.0, 0.0, -10.0);
#else	//真上から
	glTranslated(0.0, 0.0, -15.0);
	glRotated(90, 1.0, 0.0, 0.0);
#endif

	/* シーンの描画 */
	myGround(-2.0);                           /* 地面　　　 */

	glTranslated(0.0, -1.8, 0.0);
	//myCylinder(1.0, 0.2, 16);                 /* 土台　　　 */

	//glTranslated(0.0, 1.0, 0.0);
	//glTranslated(0.0, 0.6, 0.0);
	
	glRotated(-roll, 0, 0, 1);
	glRotated(-pitch, 1, 0, 0);
	glRotated(-yaw, 0, 1, 0);
		
	glTranslated(0.0, -0.6, 0.0);
	myBox(0.15, 1.2, 0.15);                     /* １番目の腕 */

	glTranslated(0.0, 1.0, 0.0);
	glRotated(90.0, 1.0, 0.0, 0.0);
	myCylinder(0.3, 0.22, 16);                 /* 関節　　　 */

	glTranslated(0.0, 0.0, -1.0);
	myBox(0.2, 0.2, 1.0);                     /* ２番目の腕 */

	glTranslated(0.0, 0.0, -1.2);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, blue);
	glutSolidSphere(0.5,18,18);                       /* ハンド　　 */

	glFlush();
}

static void resize(int w, int h)
{
	/* ウィンドウ全体をビューポートにする */
	glViewport(0, 0, w, h);

	/* 透視変換行列の指定 */
	glMatrixMode(GL_PROJECTION);

	/* 透視変換行列の初期化 */
	glLoadIdentity();
	gluPerspective(30.0, (double)w / (double)h, 1.0, 100.0);

	/* モデルビュー変換行列の指定 */
	glMatrixMode(GL_MODELVIEW);
}

static void keyboard(unsigned char key, int x, int y)
{
	/* ESC か q をタイプしたら終了 */
	if (key == '\033' || key == 'q') {
		exit(0);
	}
}

void mouse(int button, int state, int x, int y)
{
	

	switch (button) {
	case GLUT_LEFT_BUTTON:
		switch (state) {
		case GLUT_DOWN:
			
			break;
		case GLUT_UP:

			break;
		default:
			break;
		}
		break;
	case GLUT_MIDDLE_BUTTON:
		//printf("middle\n");
		break;
	case GLUT_RIGHT_BUTTON:
		switch (state) {
		case GLUT_DOWN:
			
			break;
		case GLUT_UP:

			break;
		default:
			break;
		}
		//printf("right\n");
		break;
	default:

		break;
	}

	
}

void motion(int x, int y)
{
	
	glutPostRedisplay();
}

void idle(void)
{
	glutPostRedisplay();
}

static void init(void)
{
	/* 初期設定 */
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
}

/* シリアル通信初期化 */
void initSirial(char a[]){
	char data;
	//1.ポートをオープン
	arduino = CreateFile(a, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	//2014/01/22追記　これでつながらない場合には"\\\\.\\COM7"とするとつながるかもしれません。

	if (arduino == INVALID_HANDLE_VALUE){
		printf("PORT COULD NOT OPEN\n");
		system("PAUSE");
		exit(0);
	}
	//2.送受信バッファ初期化
	Ret = SetupComm(arduino, 1024, 1024);
	if (!Ret){
		printf("SET UP FAILED\n");
		CloseHandle(arduino);
		system("PAUSE");
		exit(0);
	}
	Ret = PurgeComm(arduino, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
	if (!Ret){
		printf("CLEAR FAILED\n");
		CloseHandle(arduino);
		exit(0);
	}
	//3.基本通信条件の設定
	DCB dcb;
	GetCommState(arduino, &dcb);
	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate = 115200;
	dcb.fBinary = TRUE;
	dcb.ByteSize = 8;
	dcb.fParity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;

	Ret = SetCommState(arduino, &dcb);
	if (!Ret){
		printf("SetCommState FAILED\n");
		CloseHandle(arduino);
		system("PAUSE");
		exit(0);
	}

	//4.送信準備
	DWORD dwSendSize;
	DWORD dwErrorMask;

	//5.受信準備
	char In[20];
	DWORD lRead;
	int  itemp;
	// フロー制御関係の信号を送信（フロー制御をおこなう場合）
	Ret = EscapeCommFunction(
		arduino,    // 　通信デバイスのハンドル：CreateFile()で取得したハンドルを指定
		SETRTS // 受信可能であることを相手側に示す：RTSをアクティブにする→SETRTS
		);
	if (Ret == FALSE)   // 失敗した場合
	{
		printf("EscapeCommFunction failed.\n");
		CloseHandle(arduino);
		exit(0);
	}

	cout << "シリアル通信準備OK" << endl;
}

int main(int argc, char *argv[])
{
	glutInit(&argc, argv);
	glutInitWindowSize(700, 700);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH);
	glutCreateWindow(argv[0]);
	glutDisplayFunc(display);
	glutReshapeFunc(resize);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(idle);
	init();

	/* シリアル通信の設定 */
	initSirial("\\\\.\\COM26");

	glutMainLoop();

	printf("FINISH\n");
	CloseHandle(arduino);
	system("PAUSE");
	return 0;
}