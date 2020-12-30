/*
 Author: Haoran Zhang
 Class: ECE6122
 Last Date Modified: 11/30/2019
 Description: Final Project
 
 This the final project for ECE 6122.
 In this project, a football with 15 UAVs are rendered using OpenGL.
 16 threads are used.
 Thread 0 is used to render the scene. Other threads are used to calculate the position of UAVs.
 When the UAVs approach the surface of shpere, they generate force adjusted by a PID controller.
 */

#define GL_SILENCE_DEPRECATION

#include <GL/glut.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include "mpi.h"
#include <thread>
#include "ECE_Bitmap.h"
using namespace std;

// Initial BMP
GLuint texture[2];
BMP inBitmap;

// Initial variables used in communication
const int numElements = 6; // x, y, z, vx, vy, vz

const int rcvSize = 16 * 6; // (Main task + 15 UAVs) * numElements

double* rcvbuffer = new double[rcvSize];  // Receive buffer for AllGather

double sendBuffer[numElements];  // Send buffer in individual thread


/*
   Set flag for rendering UAVs
*/
// If initial flag = true, draw the UAVs at initial location
// If initial flag = false, draw the UAVs according to the updated position.
bool initialFlag = true;

/*
   Set up flag for controling process
*/
// The control process is divided into four stages
// In the first stage, UAV use their whole force to accelerate towards point (0,0,50). When their velocity reaches 2m/s, it goes into the second stage. The first stage is triggled by "firstStage = true"

// In first process, fire in the correct direction to accelerate
bool firstStage = true;
// In the second stage, UAV reaches maximum speed of 2 m/s. So they only keep a force to offset gravity and maintain a constant velocity towards (0,0,50). The second stage is triggled by "secondStage = true". When they reach the surface of the sphere, they go into third stage.

// In second process, only generate a force to offset gravity.
bool secondStage = false;
// In the third stage, UAV reaches the surface of the sphere. They will begin to circle around the surface. So at this stage, they will generate a force tangent to the direction pointing to center. The third stage is triggled by "closeFlag = true". After firing at a tangent direction, they transit into the forth stage, orbiting around the surface.

// If closeFlag = true, then the uav has already approach the sphere.
bool closeFlag = false;
// In the forth stage, UAV begins to orbit around the surface of the sphere. The PID controller is set up to make them remain on the surface and remain a tangent speed of 3 m/s.

// If beginOrbiting set to true, let the uavs begin orbiting around the sphere
bool beginOrbiting = false;

// Initial array to store the position of UAVs
double allLocationX[16];
double allLocationY[16];
double allLocationZ[16];

// Initial array to store the velocity of UAVs
double allVelocityX[16];
double allVelocityY[16];
double allVelocityZ[16];

/*
  Set up variables for oscillating color
 */
// Initial color to oscillate during process
int oscillateColor = 255;

// Every time color reaches 128, set decreaseFlag to false, which means color begins to increase
// On the contrary, every time color reaches 255, set decreaseFlag to true, which means color begins to decrease
bool decreaseFlag = true;

// Count steps to oscillate color
int colorCount = 0;


// Set up function for reshaping window size
void changeSize(int w, int h)
{
    float ratio = ((float)w) / ((float)h); // window aspect ratio
    glMatrixMode(GL_PROJECTION); // projection matrix is active
    glLoadIdentity(); // reset the projection
    gluPerspective(60.0, ratio, 0.1, 1000.0); // perspective transformation
    glMatrixMode(GL_MODELVIEW); // return to modelview mode
    glViewport(0, 0, w, h); // set viewport (drawing area) to entire window
}

//Convert yard to meter
//input: length in yard, output:length in meter
double yardToMeter(double yard){
    return 0.9144*yard;
}

// Draw football field
void drawFootballField(){
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture[0]);
    glBegin(GL_POLYGON);
        glTexCoord2f(0, 0);
        glVertex3f(yardToMeter(-50), yardToMeter(-26.67), 0.0);
        glTexCoord2f(1, 0);
        glVertex3f(yardToMeter(50), yardToMeter(-26.67), 0.0);
        glTexCoord2f(1, 1);
        glVertex3f(yardToMeter(50), yardToMeter(26.67), 0.0);
        glTexCoord2f(0, 1);
        glVertex3f(yardToMeter(-50), yardToMeter(26.67), 0.0);
    glEnd();
    glDisable(GL_TEXTURE_2D);
    // Draw sphere
    glColor3f(1.0, 1.0, 1.0);
    glPushMatrix();
        glTranslatef(0, 0, 50);
        glutWireSphere(10, 20, 20);
        glTranslatef(0, 0, -50);
    glPopMatrix();

    
}

// Draw UAVs
void drawUAVs(){
    // First determine the color of UAVs
    glColor3f((double)oscillateColor/255.0, 0.0, 0.0);
    // If color = 255, set decreaseflag to true to let color decrease
    if (oscillateColor == 255){
        decreaseFlag = true;
    }
    // If color = 128, set decreaseflag to false to let color increase
    if (oscillateColor == 128){
        decreaseFlag = false;
    }
    // Change color every 20 time steps
    if (colorCount % 20 == 0){
        if (decreaseFlag == true){
            oscillateColor--;
        }
        else{
            oscillateColor++;
        }
    }
    
    // If it's the first time drawing the UAV, then draw them accoring to initial position
    // At the same time, initialize allLocationx, allLocationy, allLocationz
    if (initialFlag == true){
        //Set the initalFlag to false, so next time call this function, will draw the UAVs according to updated location.
        initialFlag = false;
        double dx[] = {-yardToMeter(50.0),-yardToMeter(25.0), 0.0, yardToMeter(25.0),yardToMeter(50.0)};
        double dy[] = {-yardToMeter(26.67), 0.0, yardToMeter(26.67)};
        int count = 1;
        for (int i = 0; i < 5; ++i){
            for (int j = 0; j < 3; ++j){
                // Put the initial location in array for further movement computation
                allLocationX[count] = dx[i];
                allLocationY[count] = dy[j];
                allLocationZ[count] = 0.0;
                count++;
                // Draw the UAV at initial locations
                glPushMatrix();
                glTranslatef(dx[i],dy[j], 0.0);
                glScalef(1.0/sqrt(3), 1.0/sqrt(3), 1.0/sqrt(3));
                glutSolidTetrahedron();
            }
        }
    }
    
    // If it's not the first time, draw UAV according to updated position
    else{
        // Fill data to location and velocity arrays to help rendering and calculation
        for (int i = 1; i < 16; ++i){
            allLocationX[i] = rcvbuffer[6 * i + 0];
            allLocationY[i] = rcvbuffer[6 * i + 1];
            allLocationZ[i] = rcvbuffer[6 * i + 2];
            allVelocityX[i] = rcvbuffer[6 * i + 3];
            allVelocityY[i] = rcvbuffer[6 * i + 4];
            allVelocityZ[i] = rcvbuffer[6 * i + 5];
        }
        
        for (int i = 1; i < 16; ++i){
            glPushMatrix();
                glTranslatef(allLocationX[i],allLocationY[i], allLocationZ[i]);
                glScalef(1.0/sqrt(3), 1.0/sqrt(3), 1.0/sqrt(3));
                glutSolidTetrahedron();
            glPopMatrix();
        }

    }
}

// Draw the entire scene
void renderScene(void)
{
    
    // Clear color and depth buffers
    glClearColor(0.7, 0.7, 0.7, 1.0); // sky color is light blue
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Reset transformations
    glLoadIdentity();
    
    // Set the camera centered at (0 , -15, 90) and looking along directional
    // vector (0, 0, 50), with the z-axis pointing up
    gluLookAt(
              0, -15, 90,
              0.0, 0.0, 50,
              0.0, 0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    // Draw playground
    drawFootballField();

    // Draw UAVs
    drawUAVs();
    
    // Make it all visible
    glutSwapBuffers(); 

    MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD); // Gather information
}


void processNormalKeys(unsigned char key, int xx, int yy)
{
    glutPostRedisplay();
}


// Triger redisplay every 100ms
void timerFunction(int id)
{
    glutPostRedisplay();
    glutTimerFunc(100, timerFunction, 0);
}

// Main function for OpenGL
void mainOpenGL(int argc, char**argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(400, 400);

    glutCreateWindow(argv[0]);
    glEnable(GL_DEPTH_TEST);

    inBitmap.read("ff.bmp");
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    // Create Textures
    glGenTextures(2, texture);
    // Setup first texture
    glBindTexture(GL_TEXTURE_2D, texture[0]);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); //scale linearly when image bigger than texture

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); //scale linearly when image smalled than texture


    glTexImage2D(GL_TEXTURE_2D, 0, 3, inBitmap.bmp_info_header.width, inBitmap.bmp_info_header.height, 0,
        GL_BGR_EXT, GL_UNSIGNED_BYTE, &inBitmap.data[0]);

    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

    
    //Enable texture
    glEnable(GL_TEXTURE_2D);
    glutIgnoreKeyRepeat(1); // ignore key repeat when holding key down
    glutKeyboardFunc(processNormalKeys); // process standard key clicks
    
    glutReshapeFunc(changeSize);
    glutDisplayFunc(renderScene);
    glutTimerFunc(100, timerFunction, 0);
    glutMainLoop();
}

/*
   Set up functions to help compute poistion of UAVs
 */
// Input: velocity and acceleration
// Output: new velocity
double calculateNewVelocity(double velocity, double acceleration){
    return velocity + 0.1 * acceleration;
}

// Input: velocity, poistion, acceleration
// Output: new position
double calculateNewLocation(double velocity, double position, double accelerationn){
    return position + velocity * 0.1 + 0.5 * accelerationn*0.1*0.1;
}

// Input: force
// Output: acceleration
double calculateAcceraltion(double force){
    return force / 1;
}

// Generate a random number from 0 ~ 1, used when firing at a random direction
double generateRandomNumber(){
    return rand()/ double(RAND_MAX);
}

/*
   PID controller
*/
// Give negative feedback if UAVs go into sphere or have velocity pointing to center
// Also adjusting force to make UAVs maintain a speed of 3 m/s
void pidController(const double &distance, const double &velocityToCenter, const double &velocityx, const double &velocityy, const double &velocityz, const double & uavLocationx, const double & uavLocationy, const double & uavLocationz, double * force)
{
    // controller's parameters
    double k = 2.0, b = -9.0, c = 0.3;

    // Compute error velocity, which is velocity pointing to center and error distance which is distance away from the surface of the sphere
    double errVelocity = velocityToCenter;
    double errDistance = distance - 10;
    
    // Calculate tangent spped
    double tangSpeedx = velocityx - velocityToCenter * (0 - uavLocationx)/distance;
    double tangSpeedy = velocityy - velocityToCenter * (0 - uavLocationy)/distance;
    double tangSpeedz = velocityz - velocityToCenter * (50 - uavLocationz)/distance;
    
    
    double tangSpeed = sqrt(pow(tangSpeedx,2) + pow(tangSpeedy,2) + pow(tangSpeedz,2));
    
    // Set force to offset error
    force[0] = (0 - uavLocationx)/distance * (k * errDistance + b * errVelocity) + c * (3.0 - tangSpeed) * tangSpeedx; //5.0
    force[1] = (0 - uavLocationy)/distance * (k * errDistance + b * errVelocity) + c * (3.0 - tangSpeed) * tangSpeedy; //5.0
    force[2] = (50 - uavLocationz)/distance * (k * errDistance + b * errVelocity) + c * (3.0 - tangSpeed) * tangSpeedz; //5.0
    force[2] += 10*1;
    double f = sqrt(force[0]*force[0] + force[1]*force[1] + force[2]*force[2]);
    
    // If force is greater than 20, set the force smaller
    if(f > 20)
    {
        force[0] = force[0] / f * 20;
        force[1] = force[1] / f * 20;
        force[2] = force[2]/ f * 20;
    }
}


//Calculate the location of UAVs
void calcualteUAVsLocation(int rank){
    
    // Initial variables to get rank i's location and velocity information.
    double uavLocationx = allLocationX[rank];
    double uavLocationy = allLocationY[rank];
    double uavLocationz = allLocationZ[rank];
    double uavVelocityx = allVelocityX[rank];
    double uavVelocityy = allVelocityY[rank];
    double uavVelocityz = allVelocityZ[rank];
    double uavAcclerationx = 0;
    double uavAcclerationy = 0;
    double uavAcclerationz = 0;
    
    // Test for collision
    // If collision happens, change speed to the other UAV
    for (int i = 1; i < 16; ++i){
        if (i != rank){
            double dist = sqrt(pow(uavLocationx - allLocationX[i],2) + pow(uavLocationy - allLocationY[i],2) + pow(uavLocationz - allLocationZ[i],2));
            // If distance between two uavs is smaller than 1cm
            if (dist <= 0.01){
                uavVelocityx = allVelocityX[i];
                uavVelocityy = allVelocityY[i];
                uavVelocityz = allVelocityZ[i];
                break;
            }
        }
    }
    
    // Initial force and direction variables
    double force;
    double directionx;
    double directiony;
    double directionz;
    double uniformedDirectionx;
    double uniformedDirectiony;
    double uniformedDirectionz;
    double forcex;
    double forcey;
    double forcez;
    
    // If already approached the surface of the sphere, set closeFlag to true
    if (sqrt(pow(uavLocationx-0,2) + pow(uavLocationy-0,2) + pow(uavLocationz-50,2)) < 10){
        closeFlag = true;
    }
    // Else the UAVs haven't approached the sphere
    else {
        if ( sqrt(pow(uavVelocityx,2) + pow(uavVelocityy,2) + pow(uavVelocityz,2)) <= 2){
            // If velocity is smaller than 2, set firstStage to true
            firstStage = true;
        }
        else {
            // If velocity is larger than 2, set secondStage to true
            firstStage = false;
            secondStage = true;
        }
    }
    
    // If closeFlag is not true, so either it's in first stage or second stage
    if (closeFlag == false){
        if (firstStage == true){
            
            // Set the force to 10 Newton, which is potentiall the maximum force after subtracting gravity
            force = 10;
            
            // Compute direction pointing towards the center
            directionx = 0 - uavLocationx;
            directiony = 0 - uavLocationy;
            directionz = 50 - uavLocationz;
            
            // Normalize the direction vector
            uniformedDirectionx = directionx / sqrt(pow(directionx,2) + pow(directiony,2) + pow(directionz,2));
            uniformedDirectiony = directiony / sqrt(pow(directionx,2) + pow(directiony,2) + pow(directionz,2));
            uniformedDirectionz = directionz / sqrt(pow(directionx,2) + pow(directiony,2) + pow(directionz,2));
            
            // Break down force in x-axis, y-axis, z-axis
            forcex = force * uniformedDirectionx;
            forcey = force * uniformedDirectiony;
            forcez = force * uniformedDirectionz + 10 - 10;   //offset gravity
            
            // Calculate acceleration using force
            uavAcclerationx = calculateAcceraltion(forcex);
            uavAcclerationy = calculateAcceraltion(forcey);
            uavAcclerationz = calculateAcceraltion(forcez);
            
            // Fill the new location and velocity into arrays to prepare for transmitting
            allLocationX[rank] = calculateNewLocation(uavVelocityx, uavLocationx, uavAcclerationx);
            allLocationY[rank] = calculateNewLocation(uavVelocityy, uavLocationy, uavAcclerationy);
            allLocationZ[rank] = calculateNewLocation(uavVelocityz, uavLocationz, uavAcclerationz);
            allVelocityX[rank] = calculateNewVelocity(uavVelocityx, uavAcclerationx);
            allVelocityY[rank] = calculateNewVelocity(uavVelocityy, uavAcclerationy);
            allVelocityZ[rank] = calculateNewVelocity(uavVelocityz, uavAcclerationz);
        }
        if (secondStage == true){
            // If second stage is true, then the velocity is already 2 m/s.
            // So don't add other force other than offset of gravity to remain in uniform motion
            // Set force to zero
            force = 0;
            
            // Calculate vector pointing to center
            directionx = 0 - uavLocationx;
            directiony = 0 - uavLocationy;
            directionz = 50 - uavLocationz;
            uniformedDirectionx = directionx / sqrt(pow(directionx,2) + pow(directiony,2) + pow(directionz,2));
            uniformedDirectiony = directiony / sqrt(pow(directionx,2) + pow(directiony,2) + pow(directionz,2));
            uniformedDirectionz = directionz / sqrt(pow(directionx,2) + pow(directiony,2) + pow(directionz,2));
            
            // Calculate force
            forcex = force * uniformedDirectionx;
            forcey = force * uniformedDirectiony;
            forcez = force * uniformedDirectionz + 10 - 10;   // Add in a force to offset gravity
            
            // Calculate acceleration
            uavAcclerationx = calculateAcceraltion(forcex);
            uavAcclerationy = calculateAcceraltion(forcey);
            uavAcclerationz = calculateAcceraltion(forcez);
            
            // Fill in data
            allLocationX[rank] = calculateNewLocation(uavVelocityx, uavLocationx, uavAcclerationx);
            allLocationY[rank] = calculateNewLocation(uavVelocityy, uavLocationy, uavAcclerationy);
            allLocationZ[rank] = calculateNewLocation(uavVelocityz, uavLocationz, uavAcclerationz);
            allVelocityX[rank] = calculateNewVelocity(uavVelocityx, uavAcclerationx);
            allVelocityY[rank] = calculateNewVelocity(uavVelocityy, uavAcclerationy);
            allVelocityZ[rank] = calculateNewVelocity(uavVelocityz, uavAcclerationz);
        }
    }
    if (closeFlag == true){
        // If closeFlage is set to true, then the UAVs has approached the surface
        if (beginOrbiting == true){
            // If beginOrbiting is set to true, then use PID controller to control UAVs remain on the surface
            // Compute vector pointing to center
            double vectorToCenterx = 0 - uavLocationx;
            double vectorToCentery = 0 - uavLocationy;
            double vectorToCenterz = 50 - uavLocationz;
            
            //Compute distance to the center
            double distance = sqrt(pow(uavLocationx - 0,2) + pow(uavLocationy - 0,2) + pow(uavLocationz - 50,2));
           // Normalize
            uniformedDirectionx = vectorToCenterx / distance;
            uniformedDirectiony = vectorToCentery / distance;
            uniformedDirectionz = vectorToCenterz / distance;
            
            // Calculate how much do velocity break down on the direction pointing to center
            double velocityToCenter = uniformedDirectionx*uavVelocityx + uniformedDirectiony*uavVelocityy + uniformedDirectionz * uavVelocityz;
          
            // Declare a force vector to store the force calculated in functino.
            double f[3];
            
            // Use controller to determin how much force needed to add in
            pidController(distance, velocityToCenter, uavVelocityx, uavVelocityy, uavVelocityz, uavLocationx, uavLocationy, uavLocationz, f);
            
            // Break down force in three directino
            forcex = f[0];
            forcey = f[1];
            forcez = f[2] - 10;  //Substract gravity

            // Compute Acceleraion
            uavAcclerationx = calculateAcceraltion(forcex);
            uavAcclerationy = calculateAcceraltion(forcey);
            uavAcclerationz = calculateAcceraltion(forcez);
            
            // Fill in location and velocity
            allLocationX[rank] = calculateNewLocation(uavVelocityx, uavLocationx, uavAcclerationx);
            allLocationY[rank] = calculateNewLocation(uavVelocityy, uavLocationy, uavAcclerationy);
            allLocationZ[rank] = calculateNewLocation(uavVelocityz, uavLocationz, uavAcclerationz);
            allVelocityX[rank] = calculateNewVelocity(uavVelocityx, uavAcclerationx);
            allVelocityY[rank] = calculateNewVelocity(uavVelocityy, uavAcclerationy);
            allVelocityZ[rank] = calculateNewVelocity(uavVelocityz, uavAcclerationz);
        }
        else{
            // If closeFlag is set to true, but orbiting is not started yet, then a tangent force needs to be added to creat a tangent velocity to let the UAVs orbit around the center.
            
            // Set seed to generate random direction
            srand(3 * rank + time(NULL));
            
            // Generate two random variables
            // One is used as direction in x - axis
            // The other one is used as direction in z - axis
            double randomnumber1 = generateRandomNumber();
            double randomnumber2 = generateRandomNumber();

            // Compute vector to center
            double vectorToCenterx = 0 - uavLocationx;
            double vectorToCentery = 0 - uavLocationy;
            double vectorToCenterz = 0 - uavLocationz;

            // Since vector pointing to center and tangent vector is perpendicular to each other, so their inner product is 0. So the directino in z-axis of tangent vector can be computed.
            double thirdnumber = -(randomnumber1 * vectorToCenterx + randomnumber2 * vectorToCentery)/vectorToCenterz;

            // Normalize the tangent vector
            uniformedDirectionx = randomnumber1 / sqrt(pow(randomnumber1,2) + pow(randomnumber2,2) + pow(thirdnumber,2));
            uniformedDirectiony = randomnumber2 / sqrt(pow(randomnumber1,2) + pow(randomnumber2,2) + pow(thirdnumber,2));
            uniformedDirectionz = thirdnumber / sqrt(pow(randomnumber1,2) + pow(randomnumber2,2) + pow(thirdnumber,2));

            // Give force to the tangent direction chosen
            // Choose force to 10, since 10 is potentially the largest force subtracting the gravity
            force = 10;

            // Break down the force
            forcex = force * uniformedDirectionx;
            forcey = force * uniformedDirectiony;
            forcez = force * uniformedDirectionz + 10 - 10; //offset gravity

            // Calculate acceleration
            uavAcclerationx = calculateAcceraltion(forcex);
            uavAcclerationy = calculateAcceraltion(forcey);
            uavAcclerationz = calculateAcceraltion(forcez);

            // Fill in information fo location and velocity
            allLocationX[rank] = calculateNewLocation(uavVelocityx, uavLocationx, uavAcclerationx);
            allLocationY[rank] = calculateNewLocation(uavVelocityy, uavLocationy, uavAcclerationy);
            allLocationZ[rank] = calculateNewLocation(uavVelocityz, uavLocationz, uavAcclerationz);
            allVelocityX[rank] = calculateNewVelocity(uavVelocityx, uavAcclerationx);
            allVelocityY[rank] = calculateNewVelocity(uavVelocityy, uavAcclerationy);
            allVelocityZ[rank] = calculateNewVelocity(uavVelocityz, uavAcclerationz);

            beginOrbiting = true;
        }
        
    }
    
}

//Main program:
//use MPI to create multithread and render scene of football playground using thread 0
int main(int argc, char **argv)
{
   
    int numTasks, rank;
    
    int rc = MPI_Init(&argc, &argv);
    
    if (rc != MPI_SUCCESS)
    {
        printf("Error starting MPI program. Terminating.\n");
        MPI_Abort(MPI_COMM_WORLD, rc);
    }
    
    MPI_Comm_size(MPI_COMM_WORLD, &numTasks);
    
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    
    int gsize = 0;
    
    MPI_Comm_size(MPI_COMM_WORLD, &gsize);
    
    // Initialize location
    double dx[] = {-yardToMeter(50.0),-yardToMeter(25.0), 0.0, yardToMeter(25.0),yardToMeter(50.0)};
    double dy[] = {-yardToMeter(26.67), 0.0, yardToMeter(26.67)};
    int count = 1;
    for (int i = 0; i < 5; ++i){
        for (int j = 0; j < 3; ++j){
            // Put the initial location in array for further movement computation
            allLocationX[count] = dx[i];
            allLocationY[count] = dy[j];
            allLocationZ[count] = 0.0;
            count++;
        }
    }

    // Use thread 0 to render the scene
    if (rank == 0)
    {
        mainOpenGL(argc, argv);
    }
    // Use other threads to calculate locaitons of UAVs
    else
    {
        
        // Sleep for 5 seconds
        std::this_thread::sleep_for(std::chrono::seconds(1));  //5
        for (int ii = 0; ii < 600 ; ii++) // ii <600
        {
            calcualteUAVsLocation(rank);
            //Fill data to send buffer
            sendBuffer[0] = allLocationX[rank];
            sendBuffer[1] = allLocationY[rank];
            sendBuffer[2] = allLocationZ[rank];
            sendBuffer[3] = allVelocityX[rank];
            sendBuffer[4] = allVelocityY[rank];
            sendBuffer[5] = allVelocityZ[rank];

            MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
            //Fill data to location and velocity arrays to help rendering and calculation
            for (int i = 1; i < 16; ++i){
                allLocationX[i] = rcvbuffer[6 * i + 0];
                allLocationY[i] = rcvbuffer[6 * i + 1];
                allLocationZ[i] = rcvbuffer[6 * i + 2];
                allVelocityX[i] = rcvbuffer[6 * i + 3];
                allVelocityY[i] = rcvbuffer[6 * i + 4];
                allVelocityZ[i] = rcvbuffer[6 * i + 5];
            }
        }
    }
    return 0; // this is just to keep the compiler happy
}
