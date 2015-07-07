/* PG2SVG.c
 *
 * PolarGraph command file to SVG vector graphic
 *
 * Author: Matteo Geromin
 */

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "libmsvg\src\msvg.h"

#define TESTFILE "preview.svg"

float version = 1.0;

float  mmPerRev = 71.0;
int  motorStepsPerRev = 768.0;
float  mmPerStep = 0.092448;		// mmPerStep = mmPerRev / motorStepsPerRev
float  steps_per_mm = 21.6338;		// steps_per_mm = motorStepsPerRev / mmPerRev
int  mwidth = 722;					// space between motor pulleys
float  penWidth = 0.41;

//  Drawing direction
#define DIR_NE	1
#define DIR_SE	2
#define DIR_SW	3
#define DIR_NW	4
#define DIR_N	5
#define DIR_E	6
#define DIR_S	7
#define DIR_W	8
static int globalDrawDirection = 4; //DIR_NW;

#define DIR_MODE_AUTO		1
#define DIR_MODE_PRESET		2
#define DIR_MODE_RANDOM		3
static int globalDrawDirectionMode = 1; //DIR_MODE_AUTO;
// general defines
#define true 1
#define false 0

// define directions
#define FORWARD  0
#define BACKWARD 1

// define sides
#define SX		0
#define DX		1

FILE *output_file;
static unsigned char lastWaveWasTop = true;

int k = 0;
char points[10000] = "";
double home_x = 0,home_y = 0;
long current_pos[2] = {0 ,0};
MsvgElement *root, *son;

/////////////////////////////////////////////////////////////////////
// FUNCTIONS
/////////////////////////////////////////////////////////////////////
void to_cartesian(double *x, double *y, unsigned int Alen, unsigned int Blen);
int random(int min, int max);
void setPosition(long targetA, long targetB);
void setCurrentPosition(long Position , int side);
int maxDensity(float penSize, int rowSize);
int scaleDensity(int inDens, int inMax, int outMax);
void drawScribblePixel(long originA, long originB, int size, int density);
void drawSquarePixel_command(long originA, long originB, int size, int density);
long currentPosition(int side);
int getAutoDrawDirection(long targetA, long targetB, long sourceA, long sourceB);
void changeLength(long A, long B);
void drawSquarePixel(int length, int width, int density, int drawDirection); 
void drawSquareWaveAlongA(int waveAmplitude, int waveLength, int totalWaves, int waveNo);
void drawSquareWaveAlongB(int waveAmplitude, int waveLength, int totalWaves, int waveNo);
void moveA(long dist);
void moveB(long dist);
void flipWaveDirection();
void make_line(double x1, double y1, double x2, double y2);

// convert step count back to cartesian values
void to_cartesian(double *x, double *y, unsigned int Alen, unsigned int Blen)
{
	double a,b;
	a=Alen/steps_per_mm;	// convert counts to mm
	b=Blen/steps_per_mm;
	*x=((pow(mwidth,2)-pow(b,2)+pow(a,2))/(mwidth*2));
	*y=(sqrt( pow(a,2) - pow(*x,2)));
	*y = *y - home_y;
}

//////////////////////////////////////////////////////////////////
// Return random number from min to max							//
//////////////////////////////////////////////////////////////////
int random(int min, int max)
{
	int num = rand();
	return ((num % (max-min)) + min);
}
//////////////////////////////////////////////////////////////////
// Print a line													//
//////////////////////////////////////////////////////////////////
void make_line(double x1, double y1, double x2, double y2)
{
	char temp_char[20];
	if(x1<0 || x2<0 || y1<0 || y2<0) return;
	sprintf( temp_char, "%.3f", x2 );
	strcat(points,temp_char);
	strcat(points,",");
	sprintf(temp_char, "%.3f", y2 );
	strcat(points,temp_char);
	strcat(points,",");
}

//////////////////////////////////////////////////////////////////
// Move A (SX)													//
//////////////////////////////////////////////////////////////////
void moveA(long dist)
{
	double x1,y1,x2,y2;
	long a;
	to_cartesian(&x1,&y1,current_pos[0],current_pos[1]);
	a = current_pos[0] + dist;
	to_cartesian(&x2,&y2,a,current_pos[1]);
	make_line(x1,y1,x2,y2);
	current_pos[0] = a;
}
//////////////////////////////////////////////////////////////////
// Move B (DX)													//
//////////////////////////////////////////////////////////////////
void moveB(long dist)
{
	double x1,y1,x2,y2;
	long b;
	to_cartesian(&x1,&y1,current_pos[0],current_pos[1]);
	b = current_pos[1] + dist;
	to_cartesian(&x2,&y2,current_pos[0],b);
	make_line(x1,y1,x2,y2);
	current_pos[1] = b;
}
//////////////////////////////////////////////////////////////////
// Change lenght										//
//////////////////////////////////////////////////////////////////
void changeLength(long A, long B)
{
	double x1,y1,x2,y2;
	to_cartesian(&x1,&y1,current_pos[0],current_pos[1]);
	to_cartesian(&x2,&y2,A,B);
	make_line(x1,y1,x2,y2);
	current_pos[0] = A;
	current_pos[1] = B;
}
//////////////////////////////////////////////////////////////////
// Return current position										//
//////////////////////////////////////////////////////////////////
long currentPosition(int side)
{
	return current_pos[side];
}
//////////////////////////////////////////////////////////////////
// Flip Wave Direction											//
//////////////////////////////////////////////////////////////////
void flipWaveDirection()
{
	if (lastWaveWasTop)
		lastWaveWasTop = false;
	else
		lastWaveWasTop = true;
}
//////////////////////////////////////////////////////////////////
// Get Draw Direction											//
//////////////////////////////////////////////////////////////////
int getAutoDrawDirection(long targetA, long targetB, long sourceA, long sourceB)
{
	int dir = DIR_SE;

	if (targetA<sourceA && targetB<sourceA)
	{
		dir = DIR_NW;
	}
	else if (targetA>sourceA && targetB>sourceB)
	{
		dir = DIR_SE;
	}
	else if (targetA<sourceA && targetB>sourceB)
	{
		dir = DIR_SW;
	}
	else if (targetA>sourceA && targetB<sourceB)
	{
		dir = DIR_NE;
	}
	else if (targetA==sourceA && targetB<sourceB)
	{
		dir = DIR_NE;
	}
	else if (targetA==sourceA && targetB>sourceB)
	{
		dir = DIR_SW;
	}
	else if (targetA<sourceA && targetB==sourceB)
	{
		dir = DIR_NW;
	}
	else if (targetA>sourceA && targetB==sourceB)
	{
		dir = DIR_SE;
	}
	else
	{
		// default SE
	}
	return dir;
}
//////////////////////////////////////////////////////////////////
// Draw Square Wave A long A									//
//////////////////////////////////////////////////////////////////
void drawSquareWaveAlongA(int waveAmplitude, int waveLength, int totalWaves, int waveNo)
{
	if (waveNo == 0)
	{
		// first one, half a line and an along
		if (lastWaveWasTop) {
			moveB(waveAmplitude/2);
			moveA(waveLength);
		}
		else {
			moveB(0-(waveAmplitude/2));
			moveA(waveLength);
		}
		flipWaveDirection();
	}
	else if (waveNo == totalWaves)
	{
		// last one, half a line with no along
		if (lastWaveWasTop) {
			moveB(waveAmplitude/2);
		}
		else {
			moveB(0-(waveAmplitude/2));
		}
	}
	else
	{
		// intervening lines - full lines, and an along
		if (lastWaveWasTop) {
			moveB(waveAmplitude);
			moveA(waveLength);
		}
		else {
			moveB(0-waveAmplitude);
			moveA(waveLength);
		}
		flipWaveDirection();
	}
}
//////////////////////////////////////////////////////////////////
// Draw Square Wave A long B									//
//////////////////////////////////////////////////////////////////
void drawSquareWaveAlongB(int waveAmplitude, int waveLength, int totalWaves, int waveNo)
{
	if (waveNo == 0)
	{
		// first one, half a line and an along
		if (lastWaveWasTop) {
			moveA(waveAmplitude/2);
			moveB(waveLength);
		}
		else {
			moveA(0-(waveAmplitude/2));
			moveB(waveLength);
		}
		flipWaveDirection();
	}
	else if (waveNo == totalWaves)
	{
		// last one, half a line with no along
		if (lastWaveWasTop) {
			moveA(waveAmplitude/2);
		}
		else {
			moveA(0-(waveAmplitude/2));
		}
	}
	else
	{
		// intervening lines - full lines, and an along
		if (lastWaveWasTop) {
			moveA(waveAmplitude);
			moveB(waveLength);
		}
		else {
			moveA(0-waveAmplitude);
			moveB(waveLength);
		}
		flipWaveDirection();
	}
}
//////////////////////////////////////////////////////////////////
// Draw Scribble Pixel											//
//////////////////////////////////////////////////////////////////
void drawScribblePixel(long originA, long originB, int size, int density)
{
	long lowLimitA = originA-(size/2);
	long highLimitA = lowLimitA+size;
	long lowLimitB = originB-(size/2);
	//long highLimitB = lowLimitB+size;
	int randA;
	int randB;
	int i;

	int inc = 0;
	int currSize = size;

	changeLength(current_pos[0],current_pos[1]);

	for (i = 0; i <= density; i++)
	{
		randA = random(0, currSize);
		randB = random(0, currSize);
		changeLength(lowLimitA+randA, lowLimitB+randB);

		lowLimitA-=inc;
		highLimitA+=inc;
		currSize+=inc*2;
	}
}
//////////////////////////////////////////////////////////////////
// Draw Square Pixel											//
//////////////////////////////////////////////////////////////////
void drawSquarePixel(int length, int width, int density, int drawDirection)
{
	// work out how wide each segment should be
	int segmentLength = 0;
	int i;

	if (density > 0)
	{
		// work out some segment widths
		int basicSegLength = length / density;
		int basicSegRemainder = length % density;
		float remainderPerSegment = (float)basicSegRemainder / (float)density;
		float totalRemainder = 0.0;
		int lengthSoFar = 0;

		for (i = 0; i <= density; i++) 
		{
			totalRemainder += remainderPerSegment;

			if (totalRemainder >= 1.0)
			{
				totalRemainder -= 1.0;
				segmentLength = basicSegLength+1;
			}
			else
			{
				segmentLength = basicSegLength;
			}

			if (drawDirection == DIR_SE) {
				drawSquareWaveAlongA(width, segmentLength, density, i);
			}
			if (drawDirection == DIR_SW) {
				drawSquareWaveAlongB(width, segmentLength, density, i);
			}
			if (drawDirection == DIR_NW) {
				segmentLength = 0 - segmentLength; // reverse
			drawSquareWaveAlongA(width, segmentLength, density, i);
			}
			if (drawDirection == DIR_NE) {
				segmentLength = 0 - segmentLength; // reverse
				drawSquareWaveAlongB(width, segmentLength, density, i);
			}
			lengthSoFar += segmentLength;
		} // end of loop
	}
}
//////////////////////////////////////////////////////////////////
// Draw Square Pixel from command parameters					//
//////////////////////////////////////////////////////////////////
void drawSquarePixel_command(long originA, long originB, int size, int density)
{
	int halfSize = size / 2;

	long startPointA;
	long startPointB;
	long endPointA;
	long endPointB;

	int calcFullSize = halfSize * 2; // see if there's any rounding errors
	int offsetStart = size - calcFullSize;

	if (globalDrawDirectionMode == DIR_MODE_AUTO)
	globalDrawDirection = getAutoDrawDirection(originA, originB, currentPosition(SX), currentPosition(DX));

	if (globalDrawDirection == DIR_SE) 
	{
		startPointA = originA - halfSize;
		startPointA += offsetStart;
		startPointB = originB;
		endPointA = originA + halfSize;
		endPointB = originB;
	}
	else if (globalDrawDirection == DIR_SW)
	{
		startPointA = originA;
		startPointB = originB - halfSize;
		startPointB += offsetStart;
		endPointA = originA;
		endPointB = originB + halfSize;
	}
	else if (globalDrawDirection == DIR_NW)
	{
		startPointA = originA + halfSize;
		startPointA -= offsetStart;
		startPointB = originB;
		endPointA = originA - halfSize;
		endPointB = originB;
	}
	else //(drawDirection == DIR_NE)
	{
		startPointA = originA;
		startPointB = originB + halfSize;
		startPointB -= offsetStart;
		endPointA = originA;
		endPointB = originB - halfSize;
	}
	density = scaleDensity(density, 255, maxDensity(penWidth, size));
	changeLength(startPointA, startPointB);
	if (density > 1)
	{
		drawSquarePixel(size, size, density, globalDrawDirection);
	}
	changeLength(endPointA, endPointB);
}
//////////////////////////////////////////////////////////////////
// Set position													//
//////////////////////////////////////////////////////////////////
void setPosition(long targetA, long targetB)
{
	setCurrentPosition(targetA , SX);
	setCurrentPosition(targetB , DX);
}
//////////////////////////////////////////////////////////////////
// Set Current Position											//
//////////////////////////////////////////////////////////////////
void setCurrentPosition(long Position , int side)
{
	current_pos[side] = Position;
}
//////////////////////////////////////////////////////////////////
// get Max Density												//
//////////////////////////////////////////////////////////////////
int maxDensity(float penSize, int rowSize)
{
	float rowSizeInMM = mmPerStep * rowSize;
	float numberOfSegments = rowSizeInMM / penSize;
	int maxDens = 1;
	if (numberOfSegments >= 2.0)
		maxDens = (int)numberOfSegments;

	return maxDens;
}
//////////////////////////////////////////////////////////////////
// Scale Density												//
//////////////////////////////////////////////////////////////////
int scaleDensity(int inDens, int inMax, int outMax)
{
	float reducedDens = ((float)inDens / (float)inMax) * (float)outMax;
	reducedDens = outMax-reducedDens;
	// round up if bigger than .5
	int result = (int)(reducedDens);
	if (reducedDens - (result) > 0.5)
		result ++;

	return result;
}

int main(int argc, char *argv[])
{
	char line[256];				// input line from file
	char cmd[6];				// input command
	double	 x,y,x1,y1;			// output values
	unsigned int  ia,ib;		// input values - length of a motor string and b motor string
	int  pen_up = true;			// pen state 1=up, 0=down
	unsigned int  mina = 20000;	// keep min/max on step counts
	unsigned int  minb = 20000;
	unsigned int  maxa = 0;
	unsigned int  maxb = 0;
	unsigned int  minx = 20000;	// keep min/max on coords
	unsigned int  miny = 20000;
	unsigned int  maxx = 0;
	unsigned int  maxy = 0;
	int  size,shade;			// size and shade of image pixels
	FILE *input_file;
	int path_exists =false;
	int command = 0;
	int i;
	char temp_char[20];

	root = MsvgNewElement(EID_SVG, NULL);
	MsvgAddAttribute(root, "version", "1.2");
	MsvgAddAttribute(root, "baseProfile", "tiny");
	//MsvgAddAttribute(root, "viewBox", "0 0 400 400");

/* validate args */
	if( argc < 2 )
	{
		printf("PG2SVG v%.1f by Matteo Geromin\n\n",version);
		printf("usage: %s [machine attr] polargraph_command_file\n",argv[0]);
		printf("		-mmpr : millimeters per rev\n");
		printf("		-spr  : steps per rev\n");
		printf("		-mw   : machine width\n");
		printf("		-pw   : pen width\n");
		return(1);
	}
	//num_args = argc - 1;
	i=1;
	while(i != argc-1)
	{
		if( strncmp(argv[i],"-mmpr",5) == 0)
		{
			i++;
			mmPerRev = atof(argv[i]);
		}
		else if( strncmp(argv[i],"-spr",4) == 0)
		{
			i++;
			motorStepsPerRev = atof(argv[i]);
		}
		else if( strncmp(argv[i],"-mw",3) == 0)
		{
			i++;
			mwidth = atof(argv[i]);
		}
		else if( strncmp(argv[i],"-pw",3) == 0)
		{
			i++;
			penWidth = atof(argv[i]);
		}
		printf(argv[i]);
		printf("\n");
		i++;
	}

mmPerStep = mmPerRev/motorStepsPerRev;
steps_per_mm = motorStepsPerRev/mmPerRev;		// steps per mm on motor strings

	/* open file */
	if( (input_file = fopen(argv[i],"r")) == NULL )
	{
		printf("can't open input file\n");
		return(1);
	}

	/* read a line */
	fgets(line,sizeof(line),input_file);
	while( !feof(input_file) )
	{
		/* check for commands */
		command++;
		if(command%100 == 0) printf("Command nr. %d\n",command);
		// pen up
		if( strncmp(line,"C14",3) == 0)
		{
			pen_up = true;		// pen goes up
			if( path_exists) 
			{
				path_exists = false;
			}
		}
		// pen down
		else if( strncmp(line,"C13",3) == 0)
		{
			pen_up = false;
		}
		// set home position
		else if( strncmp(line,"C09",3) == 0)
		{
			/* obtain the parameters */
			sscanf(line,"%3s,%d,%d",cmd,&ia,&ib);
			setPosition(ia,ib);
			to_cartesian(&x,&y,ia,ib);
			home_x = x;
			home_y = y;
		}
		// set draw direction
		else if( strncmp(line,"C08",3) == 0)
		{
			/* obtain the parameters */
			sscanf(line,"%3s,%d,%d",cmd,&ia,&ib);
			globalDrawDirectionMode = ia;
			globalDrawDirection = ib;
		}
		// move or new move straight
		else if( (strncmp(line,"C01",3) == 0) || (strncmp(line,"C17",3) == 0) )
		{
			/* obtain the parameters */
			sscanf(line,"%3s,%d,%d",cmd,&ia,&ib);
			/* convert them to cartesian values */
			to_cartesian(&x,&y,ia,ib);
			/* analyze them */
			if( ia < mina ) mina=ia;
			if( ia > maxa ) maxa=ia;
			if( ib < minb ) minb=ib;
			if( ib > maxb ) maxb=ib;
			if( x < minx ) minx=x;
			if( x > maxx ) maxx=x;
			if( y < miny ) miny=y;
			if( y > maxy ) maxy=y;

			/* process them */
			if(pen_up)
			{
				current_pos[0] = ia;
				current_pos[1] = ib;
			}
			else
			{
				to_cartesian(&x1,&y1,current_pos[0],current_pos[1]);
				make_line(x1,y1,x,y);
				current_pos[0] = ia;
				current_pos[1] = ib;
			}
			path_exists=true;
		}
		// Draw Pixel
		else if(strncmp(line,"C05",3) == 0)
		{
			/* obtain the parameters */
			sscanf(line,"%3s,%d,%d,%d,%d",cmd,&ia,&ib,&size,&shade);
			/* convert them to cartesian values */
			to_cartesian(&x,&y,ia,ib);
			/* analyze them */
			if( ia < mina ) mina=ia;
			if( ia > maxa ) maxa=ia;
			if( ib < minb ) minb=ib;
			if( ib > maxb ) maxb=ib;
			if( x < minx ) minx=x;
			if( x > maxx ) maxx=x;
			if( y < miny ) miny=y;
			if( y > maxy ) maxy=y;

			if(pen_up)
			{
				current_pos[0] = ia;
				current_pos[1] = ib;
			}
			else
			{
				son = MsvgNewElement(EID_POLYLINE, root);
				MsvgAddAttribute(son, "stroke", "#000");
				sprintf( temp_char, "%.2f", penWidth*2 );
				MsvgAddAttribute(son, "stroke-width", temp_char);
				MsvgAddAttribute(son, "fill", "none");
				strcpy(points,"");

				drawSquarePixel_command(ia, ib, size, shade);
				MsvgAddAttribute(son, "points", points);
			}
		}
		// Draw Scribble Pixel
		else if(strncmp(line,"C06",3) == 0)
		{
			/* obtain the parameters */
			sscanf(line,"%3s,%d,%d,%d,%d",cmd,&ia,&ib,&size,&shade);
			/* convert them to cartesian values */
			to_cartesian(&x,&y,ia,ib);
			/* analyze them */
			if( ia < mina ) mina=ia;
			if( ia > maxa ) maxa=ia;
			if( ib < minb ) minb=ib;
			if( ib > maxb ) maxb=ib;
			if( x < minx ) minx=x;
			if( x > maxx ) maxx=x;
			if( y < miny ) miny=y;
			if( y > maxy ) maxy=y;

			if(pen_up)
			{
				current_pos[0] = ia;
				current_pos[1] = ib;
			}
			else
			{
				son = MsvgNewElement(EID_POLYLINE, root);
				MsvgAddAttribute(son, "stroke", "#000");
				sprintf( temp_char, "%.2f", penWidth );
				MsvgAddAttribute(son, "stroke-width", temp_char);
				MsvgAddAttribute(son, "fill", "none");
				strcpy(points,"");

				drawScribblePixel(ia, ib, size*1.1, scaleDensity(shade, 255, maxDensity(penWidth, size)));
				MsvgAddAttribute(son, "points", points);
			}
		}
		/* read another line */
		fgets(line,sizeof(line),input_file);
	}

	strcpy(temp_char,argv[i]);
	strcat(temp_char,".svg");
	if (!MsvgWriteSvgFile(root, temp_char)) {
		printf("Error writing %s\n", temp_char);
		return 0;
	}

	return 1;
}
