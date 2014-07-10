#include <Python.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

static PyObject *mpu6050_initialize(PyObject *self, PyObject *args);
static PyObject *mpu6050_get_quaternion(PyObject *self, PyObject *args);
static PyObject *mpu6050_get_euler(PyObject *self, PyObject *args);
static PyObject *mpu6050_get_yaw_pitch_roll(PyObject *self, PyObject *args);
static PyObject *mpu6050_get_linear_accel(PyObject *self, PyObject *args);
static PyObject *mpu6050_get_linear_accel_in_world(PyObject *self, PyObject *args);
static PyObject *mpu60050_get_x_gyro_offset(PyObject *self, PyObject *args);
static PyObject *mpu60050_get_y_gyro_offset(PyObject *self, PyObject *args);
static PyObject *mpu60050_get_z_gyro_offset(PyObject *self, PyObject *args);
static PyObject *mpu60050_set_x_gyro_offset(PyObject *self, PyObject *args);
static PyObject *mpu60050_set_y_gyro_offset(PyObject *self, PyObject *args);
static PyObject *mpu60050_set_z_gyro_offset(PyObject *self, PyObject *args);
static PyObject *mpu60050_get_x_accel_offset(PyObject *self, PyObject *args);
static PyObject *mpu60050_get_y_accel_offset(PyObject *self, PyObject *args);
static PyObject *mpu60050_get_z_accel_offset(PyObject *self, PyObject *args);
static PyObject *mpu60050_set_x_accel_offset(PyObject *self, PyObject *args);
static PyObject *mpu60050_set_y_accel_offset(PyObject *self, PyObject *args);
static PyObject *mpu60050_set_z_accel_offset(PyObject *self, PyObject *args);

static PyMethodDef methods[] = {
    {"initialize",  mpu6050_initialize, METH_NOARGS,
     "Initialize MPU6050 chip"},
    {"get_quaternion",  mpu6050_get_quaternion, METH_NOARGS,
     "Get quaternion"},
    {"get_euler",  mpu6050_get_euler, METH_NOARGS,
     "Get Euler"},
    {"get_yaw_pitch_roll",  mpu6050_get_yaw_pitch_roll, METH_NOARGS,
     "Get yaw, pitch and roll"},
    {"get_linear_accel",  mpu6050_get_linear_accel, METH_NOARGS,
     "Get linear acceleration"},
    {"get_linear_accel_in_world",  mpu6050_get_linear_accel_in_world, METH_NOARGS,
     "Get linear acceleration in world"},
    {"get_x_gyro_offset",  mpu60050_get_x_gyro_offset, METH_NOARGS,
     "Get X gyro offset"},
    {"get_y_gyro_offset",  mpu60050_get_y_gyro_offset, METH_NOARGS,
     "Get Y gyro offset"},
    {"get_z_gyro_offset",  mpu60050_get_z_gyro_offset, METH_NOARGS,
     "Get Z gyro offset"},
    {"set_x_gyro_offset",  mpu60050_set_x_gyro_offset, METH_VARARGS,
     "set X gyro offset"},
    {"set_y_gyro_offset",  mpu60050_set_y_gyro_offset, METH_VARARGS,
     "Set Y gyro offset"},
    {"set_z_gyro_offset",  mpu60050_set_z_gyro_offset, METH_VARARGS,
     "Set Z gyro offset"},
    {"get_x_accel_offset",  mpu60050_get_x_accel_offset, METH_NOARGS,
     "Get X accel offset"},
    {"get_y_accel_offset",  mpu60050_get_y_accel_offset, METH_NOARGS,
     "Get Y accel offset"},
    {"get_z_accel_offset",  mpu60050_get_z_accel_offset, METH_NOARGS,
     "Get Z accel offset"},
    {"set_x_accel_offset",  mpu60050_set_x_accel_offset, METH_VARARGS,
     "set X accel offset"},
    {"set_y_accel_offset",  mpu60050_set_y_accel_offset, METH_VARARGS,
     "Set Y accel offset"},
    {"set_z_accel_offset",  mpu60050_set_z_accel_offset, METH_VARARGS,
     "Set Z accel offset"},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

PyMODINIT_FUNC
initmpu6050(void)
{
    (void) Py_InitModule("mpu6050", methods);
}

static PyObject *
mpu6050_initialize(PyObject *self, PyObject *args)
{
    // initialize device
    // printf("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    // printf("Testing device connections...\n");
    // printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // load and configure the DMP
    // printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        // printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        // printf("DMP Initialization failed (code %d)\n", devStatus);
	PyErr_Format(PyExc_ValueError, "DMP Initialization failed (code %d)", devStatus);
	return NULL;
    }

    Py_RETURN_NONE;
}

static PyObject *
mpu6050_get_quaternion(PyObject *self, PyObject *args)
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
    {
	PyErr_SetString(PyExc_ValueError, "DMP not ready");
	return NULL;
    }

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    while(fifoCount < 24 || fifoCount == 1024) {
	if (fifoCount < 24) {
	    printf("usleep\n");
	    usleep(100000);
            fifoCount = mpu.getFIFOCount();
	} else {
            mpu.resetFIFO();
	    usleep(100000);
            fifoCount = mpu.getFIFOCount();
	}
    }

    if (fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        // printf("FIFO overflow!\n");
	PyErr_SetString(PyExc_ValueError, "FIFO overflow!");
	return NULL;
	// otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (fifoCount >= 42)
    {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	return Py_BuildValue("(f, f, f, f)", q.w, q.x, q.y, q.z);
    }
    else
    {
	PyErr_SetString(PyExc_ValueError, "");
	return NULL;
    }
}

static PyObject *
mpu6050_get_euler(PyObject *self, PyObject *args)
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
    {
	PyErr_SetString(PyExc_ValueError, "DMP not ready");
	return NULL;
    }

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    while(fifoCount < 24 || fifoCount == 1024) {
	if (fifoCount < 24) {
	    printf("usleep\n");
	    usleep(100000);
            fifoCount = mpu.getFIFOCount();
	} else {
            mpu.resetFIFO();
	    usleep(100000);
            fifoCount = mpu.getFIFOCount();
	}
    }

    if (fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        // printf("FIFO overflow!\n");
	PyErr_SetString(PyExc_ValueError, "FIFO overflow!");
	return NULL;
	// otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (fifoCount >= 42)
    {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetEuler(euler, &q);
	return Py_BuildValue("(f, f, f)", euler[0], euler[1], euler[2]);
    }
    else
    {
	PyErr_SetString(PyExc_ValueError, "");
	return NULL;
    }
}

static PyObject *
mpu6050_get_yaw_pitch_roll(PyObject *self, PyObject *args)
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
    {
	PyErr_SetString(PyExc_ValueError, "DMP not ready");
	return NULL;
    }
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    while(fifoCount < 24 || fifoCount == 1024) {
	if (fifoCount < 24) {
	    printf("usleep\n");
	    usleep(100000);
            fifoCount = mpu.getFIFOCount();
	} else {
            mpu.resetFIFO();
	    usleep(100000);
            fifoCount = mpu.getFIFOCount();
	}
    }

    if (fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        // printf("FIFO overflow!\n");
	PyErr_SetString(PyExc_ValueError, "FIFO overflow!");
	return NULL;
	// otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (fifoCount >= 42)
    {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	return Py_BuildValue("(f, f, f)", ypr[0] * 180 / M_PI, ypr[1] * 180 / M_PI, ypr[2] * 180 / M_PI);
    }
    else
    {
	PyErr_SetString(PyExc_ValueError, "");
	return NULL;
    }
}

static PyObject *
mpu6050_get_linear_accel(PyObject *self, PyObject *args)
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
    {
	PyErr_SetString(PyExc_ValueError, "DMP not ready");
	return NULL;
    }
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        // printf("FIFO overflow!\n");
	PyErr_SetString(PyExc_ValueError, "FIFO overflow");
	return NULL;
	// otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (fifoCount >= 42)
    {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	return Py_BuildValue("(i, i, i)", aaReal.x, aaReal.y, aaReal.z);
    }
    else
    {
	PyErr_SetString(PyExc_ValueError, "");
	return NULL;
    }
}

static PyObject *
mpu6050_get_linear_accel_in_world(PyObject *self, PyObject *args)
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
    {
	PyErr_SetString(PyExc_ValueError, "DMP not ready");
	return NULL;
    }
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        // printf("FIFO overflow!\n");
	PyErr_SetString(PyExc_ValueError, "FIFO overflow");
	return NULL;
	// otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (fifoCount >= 42)
    {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
	return Py_BuildValue("(i, i, i)", aaWorld.x, aaWorld.y, aaWorld.z);
    }
    else
    {
	PyErr_SetString(PyExc_ValueError, "");
	return NULL;
    }
}

static PyObject *
mpu60050_get_x_gyro_offset(PyObject *self, PyObject *args)
{
    return Py_BuildValue("b", mpu.getXGyroOffset());
}

static PyObject *
mpu60050_get_y_gyro_offset(PyObject *self, PyObject *args)
{
    return Py_BuildValue("b", mpu.getYGyroOffset());
}

static PyObject *
mpu60050_get_z_gyro_offset(PyObject *self, PyObject *args)
{
    return Py_BuildValue("b", mpu.getZGyroOffset());
}

static PyObject *
mpu60050_set_x_gyro_offset(PyObject *self, PyObject *args)
{
    unsigned char offset;
    if(!PyArg_ParseTuple(args, "b", &offset))
	return NULL;
    mpu.setXGyroOffset(offset);
    Py_RETURN_NONE;
}

static PyObject *
mpu60050_set_y_gyro_offset(PyObject *self, PyObject *args)
{
    unsigned char offset;
    if(!PyArg_ParseTuple(args, "b", &offset))
	return NULL;
    mpu.setYGyroOffset(offset);
    Py_RETURN_NONE;
}

static PyObject *
mpu60050_set_z_gyro_offset(PyObject *self, PyObject *args)
{
    unsigned char offset;
    if(!PyArg_ParseTuple(args, "b", &offset))
	return NULL;
    mpu.setZGyroOffset(offset);
    Py_RETURN_NONE;
}

static PyObject *
mpu60050_get_x_accel_offset(PyObject *self, PyObject *args)
{
    return Py_BuildValue("h", mpu.getXAccelOffset());
}

static PyObject *
mpu60050_get_y_accel_offset(PyObject *self, PyObject *args)
{
    return Py_BuildValue("h", mpu.getYAccelOffset());
}

static PyObject *
mpu60050_get_z_accel_offset(PyObject *self, PyObject *args)
{
    return Py_BuildValue("h", mpu.getZAccelOffset());
}

static PyObject *
mpu60050_set_x_accel_offset(PyObject *self, PyObject *args)
{
    unsigned char offset;
    if(!PyArg_ParseTuple(args, "h", &offset))
	return NULL;
    mpu.setXAccelOffset(offset);
    Py_RETURN_NONE;
}

static PyObject *
mpu60050_set_y_accel_offset(PyObject *self, PyObject *args)
{
    unsigned char offset;
    if(!PyArg_ParseTuple(args, "h", &offset))
	return NULL;
    mpu.setYAccelOffset(offset);
    Py_RETURN_NONE;
}

static PyObject *
mpu60050_set_z_accel_offset(PyObject *self, PyObject *args)
{
    unsigned char offset;
    if(!PyArg_ParseTuple(args, "h", &offset))
	return NULL;
    mpu.setZAccelOffset(offset);
    Py_RETURN_NONE;
}

#if 0
void loop()
{
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        printf("FIFO overflow!\n");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (fifoCount >= 42) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            printf("quat %7.2f %7.2f %7.2f %7.2f    ", q.w,q.x,q.y,q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            printf("euler %7.2f %7.2f %7.2f    ", euler[0] * 180/M_PI, euler[1] * 180/M_PI, euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            printf("ypr  %7.2f %7.2f %7.2f    ", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            printf("areal %6d %6d %6d    ", aaReal.x, aaReal.y, aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            printf("aworld %6d %6d %6d    ", aaWorld.x, aaWorld.y, aaWorld.z);
        #endif

        printf("\n");
    }
}

int main()
{
    // setup();
    // usleep(100000);
    // for (;;)
    //     loop();

    return 0;
}
#endif
