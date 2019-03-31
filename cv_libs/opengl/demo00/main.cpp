#include <GL/glut.h> //The upper-case "GL" is important
#include <cmath>

const float Pi = 3.1415926536f;

void renderSceneRectangle()
{
	glClear(GL_COLOR_BUFFER_BIT);
	glRectf(-0.2f,-0.2f,0.2f,0.2f);
	glFlush();
}

void renderSceneTriangle()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glBegin(GL_TRIANGLES);
	glVertex3f(-0.5,-0.5,0.0);
	glVertex3f(0.5,0.0,0.0);
	glVertex3f(0.0,0.5,0.0);
	glEnd();
	glutSwapBuffers();
}

void renderSceneCircle(void)
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	const int n = 20;
	const float R = 0.5f;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//glLoadIdentity();
	glBegin(GL_POLYGON);
	for(int i=0; i<n; ++i)
		glVertex2f(R*cos(2*Pi/n*i), R*sin(2*Pi/n*i));
	glEnd();
	glutSwapBuffers();
	//glFlush();
}

int main(int argc, char* argv[])
{
	glutInit(&argc, (char**) argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(100,100);
	glutInitWindowSize(960,720);
	glutCreateWindow("Hello OpenGL");
	glutDisplayFunc(&renderSceneCircle);
	glutMainLoop();//enters the GLUT event processing loop.
	return 0;
}
