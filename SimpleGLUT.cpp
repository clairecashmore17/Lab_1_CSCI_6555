#include "stdafx.h"

// standard
#include <assert.h>
#include <math.h>

#include <iostream>
#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>

// glut
#include <GL/glut.h>


using namespace std;
//================================
// global variables
//================================
// screen size
int g_screenWidth  = 0;
int g_screenHeight = 0;

// frame index
int g_frameIndex = 0;

// how many key frames are provided in text file
int num_key_frames=0;
// The index to start from in the key_frames vector (start at 1 because 0 is phantom point
int key_frame_index = 1;
// Frame rate to provide to timer
int frame_rate= 0;

string interpolate_type;
string rotation_type;


// Struct for quaternion
struct Quaternion {
	float x, y, z, w;
};

struct Euler {
	float roll, pitch, yaw;
};

// Struct for Translation
struct Translation {
	float x, y, z;
};

// Struct to hold key Frame points of matrix
struct Keyframe {
	float x, y, z;
	float xR, yR, zR, wR;
};


//Initialize Quaternion, Translation, KeyFrames vector
Quaternion q;
Euler e;
Translation t;
vector<Keyframe> keyFrames;

// Code to check the sign of the float returned in quaternion
int sign(float x) {
	//std::cout << "Checking sign of " << x << "\n";
	if (x > 0) {
		//std::cout << "Positive" << "\n";
		return 1;
	}
	else if (x < 0) {
		//std::cout << "Negative" << "\n";
		return -1;
	}
	else {
		//std::cout << "Error?" << "\n";
		return 0;
	}
}


// Original identity matrix
GLfloat g_matrix[16] = {
	1,0,0,0,
	0,1,0,0,
	0,0,1,0,
	0,0,0,1
};

// create pointer to matrix.
GLfloat* m_ptr = g_matrix;

// Function to read in file key_frames to load and interpolate.
int createVectorOfKeyFrames() {
	ifstream inputFile("key_frames.txt");

	if (!inputFile.is_open()) {
		cerr << "Error opening file!" << endl;
		return 1;
	}

	string line;
	
	//cout << "File Content: " << endl;
	

	getline(inputFile, line);
	frame_rate = stoi(line);
	getline(inputFile, line);
	num_key_frames = stoi(line);
	getline(inputFile, line);
	
	interpolate_type = line;
	

	getline(inputFile, line);
	
		rotation_type = line;
	

	while(inputFile.good()){
		Keyframe temp;
		if(rotation_type == "Q"){
			inputFile >> temp.x >> temp.y >> temp.z >> temp.xR >> temp.yR >> temp.zR >> temp.wR;
		}
		else if (rotation_type == "E") {
			inputFile >> temp.x >> temp.y >> temp.z >> temp.xR >> temp.yR >> temp.zR;
		}
		
		keyFrames.push_back(temp);
	}

	inputFile.close();
	//cout << key_frame_count << endl;
	//cout << frame_rate << endl;
	//cout << keyFrames[0].x << ' ' << keyFrames[0].y << ' ' << keyFrames[0].z << ' ' << keyFrames[0].xR << ' ' << keyFrames[0].yR << ' ' << keyFrames[0].zR << ' ' << keyFrames[0].wR << endl;
	for (Keyframe i : keyFrames) {
		cout << i.x << ' ' << i.y << ' ' << i.z << ' ' << i.xR << ' ' << i.yR << ' ' << i.zR << ' ' << i.wR << endl;
	}
	return 0;
}



// CatmullInterpolation for 1 point and re-run per point
float CatmullInterpolation(float p0, float p1, float p2, float p3, float t) {
	float tCubed = t * t * t;
	float tSquared = t * t;

	// Developed from the following Matrix where a = 0.5
	/*
					|	-a		2-a		a-2		 a	|
		|t,t^2,t^3|	|	2a		a-3		3-2a	-a	|
					|	-a		0		a		 0	|
					|	0		1		0		 0	|
	*/
	float f1 = -0.5 * tCubed + tSquared - 0.5 * t;
	float f2 = 1.5 * tCubed - 2.5 * tSquared + 1.0;
	float f3 = -1.5 * tCubed + 2.0 * tSquared + 0.5 * t;
	float f4 = 0.5 * tCubed - 0.5 * tSquared;

	//Creat the resulting point
	float x = p0 * f1 + p1 * f2 + p2 * f3 + p3 * f4;
	
	// NOTE that x is representative of ANY point (x,y,z,w)
	return x;
}

// BSpline Interpolation for 1 point and re-run per point
float BSplineInterpolation(float p0, float p1,float p2, float p3,float t) {
	float tCubed = t * t * t;
	
	float tSquared = t * t;
	
	float mt3 = (1 - t) * (1 - t) * (1 - t);

	float f1 = mt3 / 6;
	float f2 = ((3 * tCubed) - (6 * tSquared) + 4) / 6;
	float f3 = ((-3 * tCubed) + (3 * tSquared) + (3 * t) + 1) / 6;
	float f4 = mt3 / 6;

	float x = p0 * f1 + p1 * f2 + p2 * f3 + p3 * f4;
	return x;
}


// Function to convert Euler Angles to a 4x4 matrix 
GLfloat* EulerToMatrix(GLfloat roll, GLfloat pitch, GLfloat yaw, GLfloat m_ptr[16], Translation t) {

	float cos_y = cos(yaw);
	float sin_y = sin(yaw);

	float cos_p = cos(pitch);
	float sin_p = sin(pitch);

	float cos_r = cos(roll);
	float sin_r = sin(roll);

	m_ptr[0] = cos_y * cos_p;
	m_ptr[1] = (cos_y * sin_p * sin_r) - (sin_y * cos_r);
	m_ptr[2] = (cos_y * sin_p * cos_r) + (sin_y * sin_r);
	m_ptr[3] = 0;
	m_ptr[4] = sin_y * cos_p;
	m_ptr[5] = (sin_y * sin_p * sin_r) + (cos_y * cos_r);
	m_ptr[6] = (sin_y * sin_p * cos_r) - (cos_y * sin_r);
	m_ptr[7] = 0;
	m_ptr[8] = -sin_p;
	m_ptr[9] = (cos_p * sin_r);
	m_ptr[10] = (cos_p * cos_r);
	m_ptr[11] = 0;
	m_ptr[12] = t.x;
	m_ptr[13] = t.y;
	m_ptr[14] = t.z;
	m_ptr[15] = 1.0f;

	return m_ptr;
}

//Function to turn a quaternion into a 4x4 matrix.
GLfloat* QuaternionToMatrix(Translation t, Quaternion q, GLfloat m_ptr[16]) {
	// Normalize the quaternion
	const float n = 1.0f / (sqrt((q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w)));

	q.x *= n;
	q.y *= n;
	q.z *= n;
	q.w *= n;

	m_ptr[0] = 1.0f - (2.0f * (q.y * q.y)) - (2.0f * (q.z * q.z));
	m_ptr[1] = 2.0f * (q.x * q.y) - 2.0f * (q.w * q.z);
	m_ptr[2] = 2.0f * (q.x * q.z) + 2.0f * (q.w * q.y);
	m_ptr[3] = 0;
	m_ptr[4] = 2.0f * (q.x * q.y) + 2.0f * (q.w * q.z);
	m_ptr[5] = 1.0f - 2.0f * (q.x * q.x) - 2.0f * (q.z * q.z);
	m_ptr[6] = 2.0f * (q.y * q.z) - 2.0f * (q.w * q.x);
	m_ptr[7] = 0;
	m_ptr[8] = 2.0f * (q.x * q.z) - 2.0f * (q.w * q.y);
	m_ptr[9] = 2.0f * (q.y * q.z) + 2.0f * (q.w * q.x);
	m_ptr[10] = 1.0f - 2.0f * (q.x * q.x) - 2.0f * (q.y * q.y);
	m_ptr[11] = 0;
	m_ptr[12] = t.x;
	m_ptr[13] = t.y;
	m_ptr[14] = t.z;
	m_ptr[15] = 1.0f;


	 
	/*
|  m0,  m1,  m2,  m3	|
|  m4,  m5,  m6,  m7	|
|  m8,  m9,  m10, m11	|
|  m12, m13, m14, m15	|

*/
	
	
	return m_ptr;
}


// Function to convert a matrix to a quaternion
Quaternion MatrixToQuaternion(GLfloat m[16]) {
	Quaternion q;

	float zero = 0;
	/*std::cout << "x :" << m[0] << "\n";
	std::cout << "y :" << m[5] << "\n";
	std::cout << "z :" << m[10] << "\n";*/
	
	/*
	|  m0,  m1,  m2,  m3	|
	|  m4,  m5,  m6,  m7	|
	|  m8,  m9,  m10, m11	|
	|  m12, m13, m14, m15	|
	
	*/
	 

	q.w = (sqrt(std::max(zero, 1 + m[0] + m[5] + m[10]))) / 2;
	q.x = (sqrt(std::max(zero, 1 + m[0] - m[5] - m[10]))) / 2;
	q.y = (sqrt(std::max(zero, 1 - m[0] + m[5] - m[10]))) / 2;
	q.z = (sqrt(std::max(zero, 1 - m[0] - m[5] + m[10]))) / 2;

 

	q.x *= sign(q.x * (m[9] - m[6]));
	q.y *= sign(q.y * (m[2] - m[8]));
	q.z *= sign(q.z * (m[4] - m[1]));

	return q;
}




//================================
// init
//================================
void init( void ) {
	// init something before main loop...
	
}

//================================
// update
//================================
void update( void ) {
	// do something before rendering...
	
		// move onto next key_frame
		key_frame_index++;
		/*cout << key_frame_index << endl; */
	//Keep spinning around keyFrames
	if (key_frame_index >= num_key_frames - 1) {
		key_frame_index = 1;
	}

	
}
void interpolateCat(string rotation_type) {

	//Translation
	float xinterpolated = CatmullInterpolation(keyFrames[key_frame_index - 1].x, keyFrames[key_frame_index].x, keyFrames[key_frame_index + 1].x, keyFrames[key_frame_index + 2].x, 1);
	float yinterpolated = CatmullInterpolation(keyFrames[key_frame_index - 1].y, keyFrames[key_frame_index].y, keyFrames[key_frame_index + 1].y, keyFrames[key_frame_index + 2].y, 1);
	float zinterpolated = CatmullInterpolation(keyFrames[key_frame_index - 1].z, keyFrames[key_frame_index].z, keyFrames[key_frame_index + 1].z, keyFrames[key_frame_index + 2].z, 1);
	
	t.x = xinterpolated;
	t.y = yinterpolated;
	t.z = zinterpolated;


	//Rotation
	float xRinterpolated = CatmullInterpolation(keyFrames[key_frame_index - 1].xR, keyFrames[key_frame_index].xR, keyFrames[key_frame_index + 1].xR, keyFrames[key_frame_index + 2].xR, 1);
	float yRinterpolated = CatmullInterpolation(keyFrames[key_frame_index - 1].yR, keyFrames[key_frame_index].yR, keyFrames[key_frame_index + 1].yR, keyFrames[key_frame_index + 2].yR, 1);
	float zRinterpolated = CatmullInterpolation(keyFrames[key_frame_index - 1].zR, keyFrames[key_frame_index].zR, keyFrames[key_frame_index + 1].zR, keyFrames[key_frame_index + 2].zR, 1);
	if (rotation_type == "Q") {
		
		float wRinterpolated = CatmullInterpolation(keyFrames[key_frame_index - 1].wR, keyFrames[key_frame_index].wR, keyFrames[key_frame_index + 1].wR, keyFrames[key_frame_index + 2].wR, 1);
		
		//Rotation
		q.x = xRinterpolated;
		q.y = yRinterpolated;
		q.z = zRinterpolated;
		q.w = wRinterpolated;

	}
	else if (rotation_type == "E") {
		e.roll = xRinterpolated;
		e.pitch = yRinterpolated;
		e.yaw = zRinterpolated;
	}
	


	
}
void interpolateBspline(string rotation_type) {
	//Translation
	float xinterpolated = BSplineInterpolation(keyFrames[key_frame_index - 1].x, keyFrames[key_frame_index].x, keyFrames[key_frame_index + 1].x, keyFrames[key_frame_index + 2].x, 1);
	float yinterpolated = BSplineInterpolation(keyFrames[key_frame_index - 1].y, keyFrames[key_frame_index].y, keyFrames[key_frame_index + 1].y, keyFrames[key_frame_index + 2].y, 1);
	float zinterpolated = BSplineInterpolation(keyFrames[key_frame_index - 1].z, keyFrames[key_frame_index].z, keyFrames[key_frame_index + 1].z, keyFrames[key_frame_index + 2].z, 1);

	t.x = xinterpolated;
	t.y = yinterpolated;
	t.z = zinterpolated;


	//Rotation
	float xRinterpolated = BSplineInterpolation(keyFrames[key_frame_index - 1].xR, keyFrames[key_frame_index].xR, keyFrames[key_frame_index + 1].xR, keyFrames[key_frame_index + 2].xR, 1);
	float yRinterpolated = BSplineInterpolation(keyFrames[key_frame_index - 1].yR, keyFrames[key_frame_index].yR, keyFrames[key_frame_index + 1].yR, keyFrames[key_frame_index + 2].yR, 1);
	float zRinterpolated = BSplineInterpolation(keyFrames[key_frame_index - 1].zR, keyFrames[key_frame_index].zR, keyFrames[key_frame_index + 1].zR, keyFrames[key_frame_index + 2].zR, 1);
	if (rotation_type == "Q") {
		float wRinterpolated = BSplineInterpolation(keyFrames[key_frame_index - 1].wR, keyFrames[key_frame_index].wR, keyFrames[key_frame_index + 1].wR, keyFrames[key_frame_index + 2].wR, 1);

		//Rotation
		q.x = xRinterpolated;
		q.y = yRinterpolated;
		q.z = zRinterpolated;
		q.w = wRinterpolated;

	}
	else if (rotation_type == "E") {
		e.roll = xRinterpolated;
		e.pitch = yRinterpolated;
		e.yaw = zRinterpolated;
	}
}
//================================
// render
//================================
void render( void ) {
	// clear buffer
	glClearColor (0.0, 0.0, 0.0, 0.0);
	glClearDepth (1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
	
	// render state
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	// enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// light source attributes
	GLfloat LightAmbient[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightDiffuse[]	= { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat LightSpecular[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT , LightAmbient );
	glLightfv(GL_LIGHT0, GL_DIFFUSE , LightDiffuse );
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	// surface material attributes
	GLfloat material_Ka[]	= { 0.11f, 0.06f, 0.11f, 1.0f };
	GLfloat material_Kd[]	= { 0.43f, 0.47f, 0.54f, 1.0f };
	GLfloat material_Ks[]	= { 0.33f, 0.33f, 0.52f, 1.0f };
	GLfloat material_Ke[]	= { 0.1f , 0.0f , 0.1f , 1.0f };
	GLfloat material_Se		= 10;

	glMaterialfv(GL_FRONT, GL_AMBIENT	, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE	, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR	, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION	, material_Ke);
	glMaterialf (GL_FRONT, GL_SHININESS	, material_Se);

	// modelview matrix
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	//glTranslatef (0.0, 0.0, -5);
	//glRotated(g_angle, 0.0, 1.0, 0.0);
	cout << rotation_type << endl;
	if (interpolate_type == "Catmull") {
		interpolateCat(rotation_type);
	}
	else if (interpolate_type == "Bspline") {
		interpolateBspline(rotation_type);
	}
	

	if (rotation_type == "Q") {
		m_ptr = QuaternionToMatrix(t, q, g_matrix);
	}
	else if (rotation_type == "E") {
		m_ptr = EulerToMatrix(e.roll,e.pitch,e.yaw, g_matrix, t);
	}
	
	cout << m_ptr[0] << ", " << m_ptr[1] << ", " << m_ptr[2] << ", " << m_ptr[3] << '\n' << m_ptr[4] << ", " << m_ptr[5] << ", " << m_ptr[6] << ", " << m_ptr[7] << '\n' << m_ptr[8] << ", " << m_ptr[9] << ", " << m_ptr[10] << ", " << m_ptr[11] << '\n' << m_ptr[12] << ", " << m_ptr[13] << ", " << m_ptr[14] << ", " << m_ptr[15] << endl;

	// Multiply current matrix by next matrix found in interpolation
	glMultMatrixf(m_ptr);
		
	
	
	
	// render objects
	glutSolidTeapot(1.0);

	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	// swap back and front buffers
	glutSwapBuffers();
}

//================================
// keyboard input
//================================
void keyboard( unsigned char key, int x, int y ) {
}

//================================
// reshape : update viewport and projection matrix when the window is resized
//================================
void reshape( int w, int h ) {
	// screen size
	g_screenWidth  = w;
	g_screenHeight = h;	
	
	// viewport
	glViewport( 0, 0, (GLsizei)w, (GLsizei)h );

	// projection matrix
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective(45.0, (GLfloat)w/(GLfloat)h, 1.0, 2000.0);
}


//================================
// timer : triggered every 16ms ( about 60 frames per second )
//================================
void timer( int value ) {	
	// increase frame index
	g_frameIndex++;

	update();
	
	// render
	glutPostRedisplay();

	// reset timer
	// 16 ms per frame ( about 60 frames per second )
	glutTimerFunc( frame_rate, timer, 0 );
}

//================================
// main
//================================
int main( int argc, char** argv ) {
	
	
	// create opengL window
	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH );
	glutInitWindowSize( 600, 600 ); 
	glutInitWindowPosition( 100, 100 );
	glutCreateWindow( argv[0] );

	// init
	init();
	
	// set callback functions
	glutDisplayFunc( render );
	glutReshapeFunc( reshape );
	glutKeyboardFunc( keyboard );
	glutTimerFunc( 16, timer, 0 );

	createVectorOfKeyFrames();
	
	// main loop
	glutMainLoop();


	/******  Testing of Functions   ********/
	/*q.w = keyFrames[key_frame_index].wR;
	q.x = keyFrames[key_frame_index].xR;
	q.y = keyFrames[key_frame_index].yR;
	q.z = keyFrames[key_frame_index].zR;

	t.x = keyFrames[key_frame_index].x;
	t.y = keyFrames[key_frame_index].y;
	t.z = keyFrames[key_frame_index].z;*/
	//MatrixToQuaternion(g_matrix);
	//MatrixToQuaternion(g_matrix2);
	
	//EulerToMatrix(2,3,4);

	/* m_ptr = QuaternionToMatrix(t,q, g_matrix);
	cout << m_ptr[0] << ", " << m_ptr[1] << ", " << m_ptr[2] << ", " << m_ptr[3] << '\n' << m_ptr[4] << ", " << m_ptr[5] << ", " << m_ptr[6] << ", " << m_ptr[7] << '\n' << m_ptr[8] << ", " << m_ptr[9] << ", " << m_ptr[10] << ", " << m_ptr[11] << '\n' << m_ptr[12] << ", " << m_ptr[13] << ", " << m_ptr[14] << ", " << m_ptr[15] << endl;
		*/
	return 0;
}