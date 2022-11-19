#include "stdafx.h"

// standard
#include <assert.h>
#include <math.h>

// glut
#include <GL/glut.h>

//================================
// global variables
//================================
// screen size

#define PI = 3.14159265358979
int g_screenWidth  = 0;
int g_screenHeight = 0;

GLint numberofballs = 20; 

// frame index
int g_frameIndex = 0;

// angle for rotation
int g_angle = 0;


// Original Ball Position
GLfloat intiLocation[20][3] = { 
							{ -3.0, 7.0, 0.6 }, 
							{ 9.0, 8.5, 1.0 }, 
							{ 4.0, 7.2, 0.7 }, 
							{ -4.5, 6.8, 0.8}, 
							{ 3.0, 8.6, 0.0 }, 
							{ 5.0, 9.8, 0.1 }, 
							{ -4.0, 9.0, 0.1 }, 
							{ 4.0, 12.0, 0.5 }, 
							{ 0.0, 8.2, 0.0 }, 
							{ 1.0, 7.6, 0.5 } ,
							{ -2.0, 8.0, 0.7 }, 
							{ 10.0, 9.5, 1.1 }, 
							{ 5.0, 8.2, 0.8 }, 
							{ -3.5, 7.8, 0.9 }, 
							{ 4.0, 9.6, 0.1 }, 
							{ 6.0, 10.8, 0.2 }, 
							{ -3.0, 10.0, 0.2 }, 
							{ 5.0, 13.0, 0.6 }, 
							{ 1.0, 9.2, 0.1 }, 
							{ 2.0, 8.6, 0.6 }
						};

//initial ball velocity
GLfloat inivel[20][3] = {
							{ 1.5, 0, 0 }, 
							{ -1, 0, 0 },
							{ -1, 0, 0 },
							{ 1.2, 0, 0 },
							{ -1, 0, 0 }, 
							{ 0.8, 0, 0 }, 
							{ -1, 0, 0 }, 
							{ 0.5, 0, 0 }, 
							{ -1, 0, 0 }, 
							{ 0.7, 0, 0 },
							{ 0.6, 0, 0 }, 
							{ -1, 0, 0 },
							{ -1, 0, 0 },
							{ 0.3, 0, 0 },
							{ -1, 0, 0 }, 
							{ 0.4, 0, 0 }, 
							{ -1, 0, 0 }, 
							{ 0.6, 0, 0 }, 
							{ -1, 0, 0 }, 
							{ 0.7, 0, 0 }
						};


// balls' volecity after time increased
GLfloat nextvel[20][3];

// balls' original position matrix
GLfloat posi[20][3];

// balls' position after time increased by 0.03
GLfloat nextposi[20][3];

// time incrase by 0.03
GLfloat timeIncrease = 0.03f;

// gravity acceleration simulation: velocity along -y axis would in crease 2.0 units in every 0.03 time increasement
GLfloat accel[3] = { 0, -2.0, 0 }; 

//the matrix for 20 balls, each row is one M for one ball
GLfloat ballM[20][16]; 

//the matrix for single ball implementing glMatrixMult
static GLfloat M[16]; 

//coefficient of collision: velocity after collision would reduce to 0.8 of original volecity 
GLfloat e = 0.8f; 



//this is the blend function which will give Q(t)
GLfloat funcQT(GLfloat T[4], GLfloat mati[16], GLfloat controlpoints[4])
{
	GLfloat tempres[4] = { 0 };
	GLfloat Qt = 0;

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			tempres[i] += T[j] * mati[4 * i + j];

	// Calcualte Qt
	for (int i = 0; i < 4; i++)
		Qt += tempres[i] * controlpoints[i];

	return Qt;
}


//vector dot multiplication
GLfloat vec_dot(GLfloat TempV1[3], GLfloat TempV2[3])
{

	GLfloat Vres = 0.0f;
	for(int i=0; i<3; ++i)
    {
        Vres += TempV1[i]*TempV2[i];
    }
	return Vres;
}

GLfloat disti(GLfloat TempV1[3], GLfloat TempV2[3])
{
	GLfloat temp = 0;
	for(int i=0; i<3; ++i)
	{
		temp += ((TempV1[i] - TempV2[i])*(TempV1[i] - TempV2[i]));
	}
	GLfloat disti = sqrt(temp);
	return disti;
}

//use to normalize which converts a vector to unit vector
void quatToVect(GLfloat quat[7])
{
    GLfloat squa_quaterion = quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2];
	if (squa_quaterion != 0) // avoid being divided by 0
	{
		GLfloat base_quaternion = sqrt(squa_quaterion);
		quat[0] = quat[0] / base_quaternion;
		quat[1] = quat[1] / base_quaternion;
		quat[2] = quat[2] / base_quaternion;
	}
}


// detect collision between current ball and all the other balls
void ballColli(int index) 
{
	
	for (GLint i = index + 1; i < numberofballs; i++)
	{
		if (disti(posi[index], posi[i])<1.0)
		{
			GLfloat x_axis[3];
			for (int j = 0; j < 3; j++)
			{
				x_axis[j] = posi[i][j] - posi[index][j];
			}
			quatToVect(x_axis);

			GLfloat u1x[3], u1y[3];
			GLfloat tempU1 = vec_dot(x_axis, inivel[index]);
			for (GLint j = 0; j < 3; j++)
			{
				u1x[j] = tempU1*x_axis[j];
				u1y[j] = inivel[index][j] - u1x[j];
			}

			// calculate the x_axis for the other ball
			for (GLint j = 0; j < 3; j++)
			{
				x_axis[j] = posi[index][j] - posi[i][j];
			}
			quatToVect(x_axis);

			GLfloat u2x[3], u2y[3];
			GLfloat tempU2 = vec_dot(x_axis, inivel[i]);
			for (int j = 0; j < 3; j++)
			{
				u2x[j] = tempU2*x_axis[j];
				u2y[j] = inivel[i][j] - u2x[j];
			}

			// calculate the velocity
			GLfloat v1x[3], v2x[3];
			for (int j = 0; j < 3; j++)
			{
				v1x[j] = (u1x[j] + u2x[j] - (u1x[j] - u2x[j]))*0.5;
				v2x[j] = (u1x[j] + u2x[j] - (u2x[j] - u1x[j]))*0.5;
				inivel[index][j] = v1x[j] + u1y[j];
				inivel[i][j] = v2x[j] + u2y[j];
			}
			continue;
		}
	}
}


//================================
// init
//================================
void init( void ) 
{
	// init something before main loop...
	for (int j = 0; j<numberofballs; j++){
		ballM[j][0] = 1.0f;
		ballM[j][5] = 1.0f;
		ballM[j][10] = 1.0f;
		for (int i = 0; i<3; i++){
			ballM[j][12 + i] = intiLocation[j][i];
			posi[j][i] = ballM[j][12 + i];
		}
		ballM[j][15] = 1.0f;
	}
}
void floorColli(GLint index) 
{	
	if (posi[index][1]<0.5) // Floor Collision Happens
		{ 
			inivel[index][1] = -e*inivel[index][1]; 
			inivel[index][0] = inivel[index][0];
			inivel[index][2] = inivel[index][2];
		}
}

void BallMove(GLint index)
{
	floorColli(index);
	ballColli(index);
	for (int i = 0; i<3; i++)
	{
		nextvel[index][i] = inivel[index][i] + accel[i] * timeIncrease;
		inivel[index][i] = nextvel[index][i];
		nextposi[index][i] = posi[index][i] + nextvel[index][i] * timeIncrease;
		posi[index][i] = nextposi[index][i];
		ballM[index][12 + i] = nextposi[index][i];
	}

}



//will animate the ball
void BallAnim()
{
	for (int i = 0; i < numberofballs; i++)
	{
		glPushMatrix();
		BallMove(i);
		for (int j = 0; j < 16; j++)
		{
			M[j] = ballM[i][j];
		}
		glMultMatrixf(M);
		glutSolidSphere(0.5, 20, 20);
		glPopMatrix();
	}
}

//Will render the ground
void Ground()
{
	glBegin(GL_OBJECT_PLANE);
	for (GLfloat x = -100; x < 100; x += 5.0f)
	{
		glVertex3f(x, 0, -100); glVertex3f(x, 0, 100);
	}
	for (GLfloat z = -150; z < 100; z += 5.0f)
	{
		glVertex3f(-150, 0, z); glVertex3f(100, 0, z);
	}
	glEnd();
}

//================================
// update
//================================
void update( void ) {
	// do something before rendering...

	// rotation angle
	g_angle = ( g_angle + 5 ) % 360;
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
	// render objects
	Ground();
	BallAnim();
	


	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	// swap back and front buffers
	glutSwapBuffers();
}

//================================
// keyboard input
//================================
void keyboard( unsigned char key, int x, int y )
{
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
void timer( int value )
{
	glutPostRedisplay();

	// reset timer
	glutTimerFunc(16, timer, 0);
}

struct Resolution {
	int w;
	int h;

};

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

	// main loop
	glutMainLoop();

	return 0;
}
