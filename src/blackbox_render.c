#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//For msvcrt to define M_PI:
#define _USE_MATH_DEFINES
#include <math.h>

#include <getopt.h>
#include <errno.h>

#include <fcntl.h>

#ifdef WIN32
    #include <io.h>
#endif

#include <sys/stat.h>

#include <cairo.h>

#include <ft2build.h>
#include FT_FREETYPE_H
#include "embeddedfont.h"

#include "platform.h"
#include "tools.h"
#include "parser.h"
#include "datapoints.h"
#include "expo.h"
#include "imu.h"

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define MAX_MOTORS 8
#define MAX_SERVOS 8

//Controls how fast the props spin on the video
#define MOTOR_MAX_RPS 25

#define FONTSIZE_CURRENT_VALUE_LABEL 36
#define FONTSIZE_PID_TABLE_LABEL 34
#define FONTSIZE_AXIS_LABEL 34
#define FONTSIZE_FRAME_LABEL 32

#define X_POS_LABEL 8
#define X_POS_VALUE 145

#define DATAPOINTS_EXTRA_COMPUTED_FIELDS 6

typedef enum Unit {
    UNIT_RAW = 0,
    UNIT_DEGREES_PER_SEC = 1
} Unit;

static const char* const UNIT_NAME[] = {
    "raw",
    "degree"
};

typedef enum PropStyle {
    PROP_STYLE_BLADES = 0,
    PROP_STYLE_PIE_CHART = 1
} PropStyle;

static const char* const PROP_STYLE_NAME[] = {
    "blades",
    "pie"
};

typedef struct point_t {
  double x, y;
} point_t;

typedef struct color_t {
    double r, g, b;
} color_t;

typedef struct colorAlpha_t {
    double r, g, b, a;
} colorAlpha_t;

typedef struct pngRenderingTask_t {
    cairo_surface_t *surface;
    int outputLogIndex, outputFrameIndex;
} pngRenderingTask_t;

typedef struct craftDrawingParameters_t {
    int numBlades, numMotors;
    int bladeLength;
    double tipBezierWidth, tipBezierHeight;
    double motorSpacing;

    double motorX[MAX_MOTORS];
    double motorY[MAX_MOTORS];
    int motorDirection[MAX_MOTORS];

    color_t propColor[MAX_MOTORS];
} craft_parameters_t;

typedef struct renderOptions_t {
    int logNumber;
    int imageWidth, imageHeight;
    int sticksTop, sticksRight, sticksWidth;
    int craftTop, craftRight, craftWidth;
    int fps;
    int help;
    int threads;

    int plotPids, plotPidSum, plotGyros, plotMotors;
    int drawPidTable, drawSticks, drawCraft, drawTime, drawAcc;

    int pidSmoothing, gyroSmoothing, motorSmoothing;

    int bottomGraphSplitAxes;

    Unit gyroUnit;

    int gapless;
    int rawAmperage;

    PropStyle propStyle;

    //Start and end time of video in seconds offset from the beginning of the log
    uint32_t timeStart, timeEnd;

    colorAlpha_t sticksTextColor, stickColor, stickAreaColor, crosshairColor, stickTrailColor;
    int stickTrailLength, stickRadius, stickTrailRadius;

    char *filename, *outputPrefix;
} renderOptions_t;

int stickTrailCurrent[2] = {0, 0};

const double DASHED_LINE[] = {
    20.0,  /* ink */
    5.0  /* skip */
};
const int DASHED_LINE_NUM_POINTS = sizeof (DASHED_LINE) / sizeof(DASHED_LINE[0]);

const double DOTTED_LINE[] = {
    5,  /* ink */
    5  /* skip */
};
const int DOTTED_LINE_NUM_POINTS = sizeof (DOTTED_LINE) / sizeof(DOTTED_LINE[0]);

#define PID_P 0
#define PID_I 1
#define PID_D 2
#define PID_TOTAL 3

typedef struct fieldIdentifications_t {
    int numMotors;
    color_t motorColors[MAX_MOTORS];

    bool hasPIDs;
    color_t PIDAxisColors[3][3]; //PID, axis
    int PIDLineStyle[3]; //Indexed by PID_P etc

    bool hasGyros;
    color_t gyroColors[3];

    bool hasAccs;
    color_t accColors[3];

    bool hasMagADC;

    int numServos;
    color_t servoColors[MAX_SERVOS];

    int numCells;

    // Indexes of fields we compute from the data:
    int roll, pitch, heading;
    int axisPIDSum[3];
    int cumulativeCurrent;
} fieldIdentifications_t;

color_t lineColors[] = {
    {0.984, 0.502,  0.447},
    {0.553, 0.827,  0.78},
    {1, 1,  0.702},
    {0.745, 0.729,  0.855},
    {0.502, 0.694,  0.827},
    {0.992, 0.706,  0.384},
    {0.702, 0.871,  0.412},
    {0.988, 0.804,  0.898},
    {0.851, 0.851,  0.851},
    {0.737, 0.502,  0.741},
    {0.8,   0.922,  0.773},
    {1, 0.929,  0.435}
};

const color_t WHITE = {.r = 1, .g = 1, .b = 1};

#define NUM_LINE_COLORS (sizeof(lineColors) / sizeof(lineColors[0]))

static const colorAlpha_t craftColor = {0.3, 0.3, 0.3, 1};

static const renderOptions_t defaultOptions = {
    .imageWidth = 1920, .imageHeight = 1080,
    .fps = 30, .help = 0, .threads = 3, .propStyle = PROP_STYLE_PIE_CHART,
    .plotPids = false, .plotPidSum = false, .plotGyros = true, .plotMotors = true,
    .pidSmoothing = 4, .gyroSmoothing = 2, .motorSmoothing = 2,
    .drawCraft = true, .drawPidTable = true, .drawSticks = true, .drawTime = true,
    .drawAcc = true,
    .sticksTop = 0, .sticksRight = 0, .sticksWidth = 0,
    .craftTop = 0, .craftRight = 0, .craftWidth = 0,
    .gyroUnit = UNIT_RAW,
    .filename = 0,
    .timeStart = 0, .timeEnd = 0,
    .logNumber = 0,
    .gapless = 0,
    .rawAmperage = 0,
    .sticksTextColor = {1, 1, 1, 1},
    .stickColor = {1, 0.4, 0.4, 1.0},
    .stickAreaColor = {0.3, 0.3, 0.3, 0.8},
    .crosshairColor = {0.75, 0.75, 0.75, 0.5},
    .stickTrailColor = {1, 1, 1, 1},
    .stickTrailLength = 0,
    .stickRadius = 0,
    .stickTrailRadius = 0,
};

//Cairo doesn't include this in any header (apparently it is considered private?)
extern cairo_font_face_t* cairo_ft_font_face_create_for_ft_face(FT_Face face, int load_flags);

static renderOptions_t options;
static expoCurve_t *pitchStickCurve, *pidCurve, *gyroCurve, *accCurve, *motorCurve, *servoCurve;

static semaphore_t pngRenderingSem;
static bool pngRenderingSemCreated = false;

static flightLog_t *flightLog;
static datapoints_t *points;
static int selectedLogIndex;

//Information about fields we have classified
static fieldIdentifications_t fieldMeta;

static FT_Library freetypeLibrary;

static uint32_t syncBeepTime = -1;

static point_t *stickTrails[2];

void loadFrameIntoPoints(flightLog_t *log, bool frameValid, int64_t *frame, uint8_t frameType, int fieldCount, int frameOffset, int frameSize)
{
    (void) log;
    (void) frameSize;
    (void) frameOffset;
    (void) frameType;
    (void) fieldCount;

    if (frameType == 'P' || frameType == 'I') {
        if (frameValid) {
            datapointsAddFrame(points, frame[FLIGHT_LOG_FIELD_INDEX_TIME], frame);
        } else {
            datapointsAddGap(points);
        }
    }
}

void onLogEvent(flightLog_t *log, flightLogEvent_t *event)
{
    (void) log;

    switch (event->event) {
        case FLIGHT_LOG_EVENT_SYNC_BEEP:
            syncBeepTime = event->data.syncBeep.time;
        break;
        default:
            ;
    }
}

void updateFieldMetadata()
{
    int motorGraphColorIndex = 0;

    fieldMeta.numMotors = 0;
    fieldMeta.numServos = 0;

    fieldMeta.hasGyros = flightLog->mainFieldIndexes.gyroADC[0] > -1;
    fieldMeta.hasAccs = flightLog->mainFieldIndexes.accSmooth[0] > -1;
    fieldMeta.hasMagADC = flightLog->mainFieldIndexes.magADC[0] > -1;

    fieldMeta.hasPIDs = flightLog->mainFieldIndexes.pid[0][0] > -1;

    if (fieldMeta.hasPIDs) {
        for (int axisIndex = 0; axisIndex < 3; axisIndex++) {
            if (options.plotPids) {
                fieldMeta.PIDAxisColors[PID_P][axisIndex] = lineColors[PID_P];
                fieldMeta.PIDAxisColors[PID_I][axisIndex] = lineColors[PID_I];
                fieldMeta.PIDAxisColors[PID_D][axisIndex] = lineColors[PID_D];
            } else {
                fieldMeta.PIDAxisColors[PID_P][axisIndex] = WHITE;
                fieldMeta.PIDAxisColors[PID_I][axisIndex] = WHITE;
                fieldMeta.PIDAxisColors[PID_D][axisIndex] = WHITE;
            }

            fieldMeta.PIDLineStyle[axisIndex] = 0; //TODO
        }
    }

    for (int motorIndex = 0; motorIndex < MAX_MOTORS; motorIndex++) {
        if (flightLog->mainFieldIndexes.motor[motorIndex] > -1) {
            fieldMeta.motorColors[motorIndex] = lineColors[(motorGraphColorIndex++) % NUM_LINE_COLORS];
            fieldMeta.numMotors++;
        }
    }

    for (int axisIndex = 0; axisIndex < 3; axisIndex++) {
        if (flightLog->mainFieldIndexes.gyroADC[axisIndex]) {
            if (options.plotGyros) {
                if (options.bottomGraphSplitAxes)
                    fieldMeta.gyroColors[axisIndex] = lineColors[(PID_D + 2) % NUM_LINE_COLORS];
                else
                    fieldMeta.gyroColors[axisIndex] = lineColors[axisIndex % NUM_LINE_COLORS];
            } else
                fieldMeta.gyroColors[axisIndex] = WHITE;
        }
    }

    for (int axisIndex = 0; axisIndex < 3; axisIndex++) {
        if (flightLog->mainFieldIndexes.accSmooth[axisIndex]) {
            fieldMeta.accColors[axisIndex] = lineColors[axisIndex % NUM_LINE_COLORS];
        }
    }

    for (int servoIndex = 0; servoIndex < MAX_SERVOS; servoIndex++) {
        if (flightLog->mainFieldIndexes.servo[servoIndex] > -1) {
            fieldMeta.numServos++;
            fieldMeta.servoColors[servoIndex] = lineColors[(motorGraphColorIndex++) % NUM_LINE_COLORS];
        }
    }

    if (flightLog->mainFieldIndexes.vbatLatest > -1) {
        fieldMeta.numCells = flightLogEstimateNumCells(flightLog);
    }
}

void drawCommandSticks(int64_t *frame, int imageWidth, int imageHeight, cairo_t *cr)
{
    double rcCommand[4] = {0, 0, 0, 0};
    int stickSurroundRadius = imageHeight / 11;
    if(options.sticksWidth > 0) {
      stickSurroundRadius = options.sticksWidth;
    }
    const int stickSpacing = stickSurroundRadius * 3;
    const int yawStickMax = 500;
    int stickIndex;

    int stickRadius = stickSurroundRadius / 5;
    int stickTrailRadius = stickRadius;

    if(options.stickRadius > 0) {
      stickRadius = options.stickRadius;
    }

    if(options.stickTrailRadius > 0) {
      stickTrailRadius = options.stickTrailRadius;
    }

    char stickLabel[16];
    cairo_text_extents_t extent;

    (void) imageWidth;

    for (stickIndex = 0; stickIndex < 4; stickIndex++) {
        //Check that stick data is present to be drawn:
        if (flightLog->mainFieldIndexes.rcCommand[stickIndex] < 0)
            return;

        rcCommand[stickIndex] = frame[flightLog->mainFieldIndexes.rcCommand[stickIndex]];
    }

    //Compute the position of the sticks in the range [-1..1] (left stick x, left stick y, right stick x, right stick y)
    double stickPositions[4];

    stickPositions[0] = -rcCommand[2] / yawStickMax; //Yaw
    stickPositions[1] = (1500 - rcCommand[3]) / 500; //Throttle
    stickPositions[2] = expoCurveLookup(pitchStickCurve, rcCommand[0]); //Roll
    stickPositions[3] = expoCurveLookup(pitchStickCurve, -rcCommand[1]); //Pitch

    for (stickIndex = 0; stickIndex < 4; stickIndex++) {
        //Clamp to [-1..1]
        stickPositions[stickIndex] = stickPositions[stickIndex] > 1 ? 1 : (stickPositions[stickIndex] < -1 ? -1 : stickPositions[stickIndex]);

        //Scale to our stick size
        stickPositions[stickIndex] *= stickSurroundRadius;
    }

    cairo_save(cr);

    cairo_translate(cr, -stickSpacing / 2, 0);

    //For each stick
    for (int i = 0; i < 2; i++) {
        //Fill in background
        cairo_set_source_rgba(cr, options.stickAreaColor.r, options.stickAreaColor.g, options.stickAreaColor.b, options.stickAreaColor.a);
        cairo_rectangle(cr, -stickSurroundRadius, -stickSurroundRadius, stickSurroundRadius * 2, stickSurroundRadius * 2);
        cairo_fill(cr);

        //Draw crosshair
        cairo_set_line_width(cr, 1);
        cairo_set_source_rgba(cr, options.crosshairColor.r, options.crosshairColor.g, options.crosshairColor.b, options.crosshairColor.a);
        cairo_move_to(cr, -stickSurroundRadius, 0);
        cairo_line_to(cr, stickSurroundRadius, 0);
        cairo_move_to(cr, 0, -stickSurroundRadius);
        cairo_line_to(cr, 0, stickSurroundRadius);
        cairo_stroke(cr);

        //Draw trail
        for(int j = 0; j < stickTrailCurrent[i]; j++) {
          point_t current = stickTrails[i][j];

          cairo_set_source_rgba(cr, options.stickTrailColor.r, options.stickTrailColor.g, options.stickTrailColor.b, options.stickTrailColor.a - (options.stickTrailColor.a - (j / (stickTrailCurrent[i] + 1.0))));
          cairo_arc(cr, current.x, current.y, stickTrailRadius, 0, 2 * M_PI);
          cairo_fill(cr);

          if(j > 0) {
            stickTrails[i][j-1] = stickTrails[i][j];
          }
        }

        //Draw circle to represent stick position
        double stickX = stickPositions[i * 2 + 0];
        double stickY = stickPositions[i * 2 + 1];

        if(stickTrailCurrent[i] < options.stickTrailLength) {
          stickTrailCurrent[i]++;
        }

        if(stickTrailCurrent[i] > 0) {
          point_t p = {stickX, stickY};
          stickTrails[i][stickTrailCurrent[i] - 1] = p;
        }

        cairo_set_source_rgba(cr, options.stickColor.r, options.stickColor.g, options.stickColor.b, options.stickColor.a);
        cairo_arc(cr, stickX, stickY, stickRadius, 0, 2 * M_PI);
        cairo_fill(cr);

        cairo_set_source_rgba(cr, options.sticksTextColor.r, options.sticksTextColor.g, options.sticksTextColor.b, options.sticksTextColor.a);
        cairo_set_font_size(cr, FONTSIZE_CURRENT_VALUE_LABEL);

        //Draw horizontal stick label
        int32_t labelValue;

        labelValue = frame[flightLog->mainFieldIndexes.rcCommand[(1 - i) * 2 + 0]];

        snprintf(stickLabel, sizeof(stickLabel), "%d", labelValue);
        cairo_text_extents(cr, stickLabel, &extent);

        cairo_move_to(cr, -extent.width / 2, stickSurroundRadius + extent.height + 8);
        cairo_show_text(cr, stickLabel);

        //Draw vertical stick label
        snprintf(stickLabel, sizeof(stickLabel), "%" PRId64, frame[flightLog->mainFieldIndexes.rcCommand[(1 - i) * 2 + 1]]);
        cairo_text_extents(cr, stickLabel, &extent);

        cairo_move_to(cr, -stickSurroundRadius - extent.width - 8, extent.height / 2);
        cairo_show_text(cr, stickLabel);

        //Advance to next stick
        cairo_translate(cr, stickSpacing, 0);
    }

    cairo_restore(cr);
}

/*
 * Draw a vertically-oriented propeller at the origin with the current source color
 */
void drawPropeller(cairo_t *cr, craft_parameters_t *parameters)
{
    cairo_move_to(cr, 0, 0);

    for (int i = 0; i < parameters->numBlades; i++) {
        cairo_curve_to(cr, parameters->tipBezierWidth, -parameters->tipBezierHeight, parameters->tipBezierWidth, parameters->bladeLength + parameters->tipBezierHeight, 0, parameters->bladeLength);
        cairo_curve_to(cr, -parameters->tipBezierWidth, parameters->bladeLength + parameters->tipBezierHeight, -parameters->tipBezierWidth, -parameters->tipBezierHeight, 0, 0);

        cairo_rotate(cr, (M_PI * 2) / parameters->numBlades);
    }

    cairo_fill(cr);
}

/**
 * Draw a craft with spinning blades at the origin
 */
void drawCraft(cairo_t *cr, int64_t *frame, int64_t timeElapsedMicros, craft_parameters_t *parameters)
{
    static double propAngles[MAX_MOTORS] = {0};

    double angularSpeed[MAX_MOTORS];
    double rotationThisFrame[MAX_MOTORS];
    int onionLayers[MAX_MOTORS];
    int motorIndex, onion;
    double opacity;

    char motorLabel[16];
    cairo_text_extents_t extent;

    /*if (fieldMeta.heading) {
        cairo_rotate(cr, intToFloat(frame[fieldMeta.heading]));
    }*/

    //Draw arms
    cairo_set_line_width(cr, parameters->bladeLength * 0.30);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
    cairo_set_source_rgba(cr, craftColor.r, craftColor.g, craftColor.b, craftColor.a);

    for (motorIndex = 0; motorIndex < parameters->numMotors; motorIndex++) {
        cairo_move_to(cr, 0, 0);

        cairo_line_to(
            cr,
            parameters->motorSpacing * parameters->motorX[motorIndex] * 1.2,
            parameters->motorSpacing * parameters->motorY[motorIndex] * 1.2
        );
    }

    cairo_stroke(cr);

    //Draw the central hub
    cairo_move_to(cr, 0, 0);
    cairo_arc(cr, 0, 0, parameters->motorSpacing * 0.4, 0, 2 * M_PI);
    cairo_fill(cr);

    //Compute prop speed and position
    for (motorIndex = 0; motorIndex < parameters->numMotors; motorIndex++) {
        if (flightLog->mainFieldIndexes.motor[motorIndex] > -1) {
            double scaled = doubleMax(frame[flightLog->mainFieldIndexes.motor[motorIndex]] - (int32_t) flightLog->sysConfig.motorOutputLow, 0) / (flightLog->sysConfig.motorOutputHigh - flightLog->sysConfig.motorOutputLow);

            //If motors are armed (above minthrottle), keep them spinning at least a bit
            if (scaled > 0)
                scaled = scaled * 0.9 + 0.1;

            angularSpeed[motorIndex] = scaled * M_PI * 2 * MOTOR_MAX_RPS;

            rotationThisFrame[motorIndex] = angularSpeed[motorIndex] * timeElapsedMicros / 1000000;

            // Don't need to draw as many onion layers if we aren't rotating very far
            onionLayers[motorIndex] = (int) (doubleAbs(rotationThisFrame[motorIndex]) * 10);
            if (onionLayers[motorIndex] < 1)
                onionLayers[motorIndex] = 1;
        }
    }

    cairo_set_font_size(cr, FONTSIZE_CURRENT_VALUE_LABEL);

    for (motorIndex = 0; motorIndex < parameters->numMotors; motorIndex++) {
        cairo_save(cr);
        {
            //Move to the motor centre
            cairo_translate(
                cr,
                parameters->motorSpacing * parameters->motorX[motorIndex],
                parameters->motorSpacing * parameters->motorY[motorIndex]
            );

            if (options.propStyle == PROP_STYLE_BLADES) {
                //Draw several copies of the prop along its movement path so we can simulate motion blur
                for (onion = 1; onion <= onionLayers[motorIndex]; onion++) {
                    cairo_save(cr);
                    {
                        // Opacity falls when the motor is spinning closer to max speed
                        opacity = 1.0 / (onionLayers[motorIndex] / 2.0);

                        cairo_set_source_rgba(
                            cr,
                            parameters->propColor[motorIndex].r,
                            parameters->propColor[motorIndex].g,
                            parameters->propColor[motorIndex].b,
                            /* Fade in the blade toward its rotational direction, but don't fade to zero */
                            opacity * ((((double) onion / onionLayers[motorIndex]) + 1.0) / 2)
                        );

                        cairo_rotate(cr, (propAngles[motorIndex] + (rotationThisFrame[motorIndex] * onion) / onionLayers[motorIndex]) * parameters->motorDirection[motorIndex]);

                        drawPropeller(cr, parameters);
                    }
                    cairo_restore(cr);
                }
            } else {
                cairo_set_source_rgba(
                    cr,
                    parameters->propColor[motorIndex].r / 2,
                    parameters->propColor[motorIndex].g / 2,
                    parameters->propColor[motorIndex].b / 2,
                    0.5
                );

                cairo_move_to(cr, 0, 0);
                cairo_arc(cr, 0, 0, parameters->bladeLength, 0, M_PI * 2);
                cairo_fill(cr);

                cairo_set_source_rgba(
                    cr,
                    parameters->propColor[motorIndex].r,
                    parameters->propColor[motorIndex].g,
                    parameters->propColor[motorIndex].b,
                    1
                );

                cairo_move_to(cr, 0, 0);

                /* Attempting to plot super high (corrupt) values for motors results in cairo_arc() taking forever
                 * (as it spins round and round the axis I bet) so be careful to clamp the angle properly.
                 */
                cairo_arc(cr, 0, 0, parameters->bladeLength, -M_PI_2, -M_PI_2 + M_PI * 2 *
                    doubleMin(doubleMax((double) (frame[flightLog->mainFieldIndexes.motor[motorIndex]] - (int32_t) flightLog->sysConfig.motorOutputLow) / (flightLog->sysConfig.motorOutputHigh - flightLog->sysConfig.motorOutputLow), 0.0), 1.0));
                cairo_fill(cr);
            }

            snprintf(motorLabel, sizeof(motorLabel), "%" PRId64, frame[flightLog->mainFieldIndexes.motor[motorIndex]]);

            cairo_text_extents(cr, motorLabel, &extent);

            if (parameters->motorX[motorIndex] > 0)
                cairo_translate(cr, parameters->bladeLength + 10, 0);
            else
                cairo_translate(cr, -(parameters->bladeLength + 10 + extent.width), 0);

            cairo_move_to(cr, 0, 0);

            cairo_set_source_rgb(
                cr,
                doubleMin(parameters->propColor[motorIndex].r * 1.25, 1),
                doubleMin(parameters->propColor[motorIndex].g * 1.25, 1),
                doubleMin(parameters->propColor[motorIndex].b * 1.25, 1)
            );

            cairo_show_text (cr, motorLabel);
        }
        cairo_restore(cr);
    }

    for (motorIndex = 0; motorIndex < parameters->numMotors; motorIndex++)
        propAngles[motorIndex] += rotationThisFrame[motorIndex];
}

void decideCraftParameters(craft_parameters_t *parameters, int imageWidth, int imageHeight)
{
    (void) imageHeight;

    parameters->numMotors = fieldMeta.numMotors == 3 || fieldMeta.numMotors == 4 ? fieldMeta.numMotors : 4;
    parameters->numBlades = 2;
    parameters->bladeLength = imageWidth / 25;
    if(options.craftWidth > 0) {
      parameters->bladeLength = options.craftWidth;
    }
    parameters->tipBezierWidth = 0.2 * parameters->bladeLength;
    parameters->tipBezierHeight = 0.1 * parameters->bladeLength;
    parameters->motorSpacing = parameters->bladeLength * 1.15;

    switch (parameters->numMotors) {
        case 3:
            parameters->motorX[0] = 0;
            parameters->motorY[0] = 1.41;
            parameters->motorX[1] = 1;
            parameters->motorY[1] = -1;
            parameters->motorX[2] = -1;
            parameters->motorY[2] = -1;

            parameters->motorDirection[0] = -1;
            parameters->motorDirection[1] = -1;
            parameters->motorDirection[2] = -1;
        break;
        case 4:
        default:
            parameters->motorX[0] = 1;
            parameters->motorY[0] = 1;
            parameters->motorX[1] = 1;
            parameters->motorY[1] = -1;
            parameters->motorX[2] = -1;
            parameters->motorY[2] = 1;
            parameters->motorX[3] = -1;
            parameters->motorY[3] = -1;

            parameters->motorDirection[0] = 1;
            parameters->motorDirection[1] = -1;
            parameters->motorDirection[2] = -1;
            parameters->motorDirection[3] = 1;
        break;
    }

    //TODO we can let the user choose their prop colours to match their model if they like
    for (int i = 0; i < parameters->numMotors; i++)
        parameters->propColor[i] = fieldMeta.motorColors[i];
}

/**
 * Plot the given field within the specified time period. When the output from the curve applied to a field
 * value reaches 1.0 it'll be drawn plotHeight pixels away from the origin.
 */
void plotLine(cairo_t *cr, color_t color, int64_t windowStartTime, int64_t windowEndTime, int firstFrameIndex,
        int fieldIndex, expoCurve_t *curve, int plotHeight)
{
    static const int GAP_WARNING_BOX_RADIUS = 4;
    uint32_t windowWidthMicros = (uint32_t) (windowEndTime - windowStartTime);
    int64_t fieldValue;
    int64_t frameTime;

    bool drawingLine = false;
    double lastX, lastY;

    //Draw points from this line until we leave the window
    for (int frameIndex = firstFrameIndex; frameIndex < points->frameCount; frameIndex++) {
        datapointsGetFieldAtIndex(points, frameIndex, fieldIndex, &fieldValue);
        datapointsGetTimeAtIndex(points, frameIndex, &frameTime);

        double nextX, nextY;

        nextY = (double) -expoCurveLookup(curve, fieldValue) * plotHeight;
        nextX = (double)(frameTime - windowStartTime) / windowWidthMicros * options.imageWidth;

        if (drawingLine) {
            if (!options.gapless && datapointsGetGapStartsAtIndex(points, frameIndex - 1)) {
                //Draw a warning box at the beginning and end of the gap to mark it
                cairo_rectangle(cr, lastX - GAP_WARNING_BOX_RADIUS, lastY - GAP_WARNING_BOX_RADIUS, GAP_WARNING_BOX_RADIUS * 2, GAP_WARNING_BOX_RADIUS * 2);
                cairo_rectangle(cr, nextX - GAP_WARNING_BOX_RADIUS, nextY - GAP_WARNING_BOX_RADIUS, GAP_WARNING_BOX_RADIUS * 2, GAP_WARNING_BOX_RADIUS * 2);

                cairo_move_to(cr, nextX, nextY);
            } else {
                cairo_line_to(cr, nextX, nextY);
            }
        } else {
            cairo_move_to(cr, nextX, nextY);
        }

        drawingLine = true;
        lastX = nextX;
        lastY = nextY;

        if (frameTime >= windowEndTime)
            break;
    }

    cairo_set_source_rgb(cr, color.r, color.g, color.b);
    cairo_stroke(cr);
}

void drawPIDTable(cairo_t *cr, int64_t *frame)
{
    cairo_font_extents_t fontExtent;

    cairo_font_extents(cr, &fontExtent);

    const double INTERROW_SPACING = 32;
    const double VERT_SPACING = fontExtent.height + INTERROW_SPACING;
    const double FIRST_ROW_TOP = fontExtent.height + INTERROW_SPACING;
    const double HORZ_SPACING = 100, FIRST_COL_LEFT = 140;

    const double HORZ_EXTENT = FIRST_COL_LEFT + HORZ_SPACING * 5 - 30;
    const double VERT_EXTENT = FIRST_ROW_TOP + fontExtent.height * 3 + INTERROW_SPACING * 2;

    const double PADDING = 32;

    char fieldLabel[16];
    int pidType, axisIndex;
    const char *pidName;

    cairo_save(cr);

    //Centre about the origin
    cairo_translate(cr, -HORZ_EXTENT / 2, -VERT_EXTENT / 2);

    //Draw a background box
    cairo_set_source_rgba(cr, 0, 0, 0, 0.33);

    cairo_rectangle(cr, -PADDING, -PADDING, HORZ_EXTENT + PADDING * 2, VERT_EXTENT + PADDING * 2);

    cairo_fill(cr);

    cairo_set_font_size(cr, FONTSIZE_PID_TABLE_LABEL);
    cairo_set_source_rgb(cr, 1, 1, 1);

    //Draw field labels first
    for (pidType = PID_P - 1; pidType <= PID_TOTAL; pidType++) {
        switch (pidType) {
            case PID_P - 1:
                pidName = "Gyro";
            break;
            case PID_P:
                pidName = "P";
            break;
            case PID_I:
                pidName = "I";
            break;
            case PID_D:
                pidName = "D";
            break;
            case PID_TOTAL:
                pidName = "Sum";
            break;
            default:
                pidName = "";
        }
        cairo_move_to (cr, (pidType + 1) * HORZ_SPACING + FIRST_COL_LEFT, fontExtent.height);
        cairo_show_text (cr, pidName);
    }

    for (axisIndex = 0; axisIndex < 3; axisIndex++) {
        switch (axisIndex) {
            case 0:
                pidName = "Roll";
            break;
            case 1:
                pidName = "Pitch";
            break;
            case 2:
                pidName = "Yaw";
            break;
            default:
                pidName = "";
        }

        cairo_move_to (cr, 0, FIRST_ROW_TOP + axisIndex * VERT_SPACING + fontExtent.height);
        cairo_show_text (cr, pidName);
    }

    //Now draw the values
    for (pidType = PID_P - 1; pidType <= PID_TOTAL; pidType++) {
        for (axisIndex = 0; axisIndex < 3; axisIndex++) {
            int32_t fieldValue;

            if (pidType == PID_P - 1) {
                if (fieldMeta.hasGyros) {
                    fieldValue = frame[flightLog->mainFieldIndexes.gyroADC[axisIndex]];

                    if (options.gyroUnit == UNIT_DEGREES_PER_SEC) {
                        fieldValue = (int32_t) round(flightlogGyroToRadiansPerSecond(flightLog, fieldValue) * (180 / M_PI));
                    }
                } else
                    fieldValue = 0;
            } else if (fieldMeta.hasPIDs) {
                if (pidType == PID_TOTAL) {
                    fieldValue = frame[flightLog->mainFieldIndexes.pid[PID_P][axisIndex]]
                        + frame[flightLog->mainFieldIndexes.pid[PID_I][axisIndex]]
                        + (flightLog->mainFieldIndexes.pid[PID_D][axisIndex] > -1 ? frame[flightLog->mainFieldIndexes.pid[PID_D][axisIndex]] : 0);
                } else if (flightLog->mainFieldIndexes.pid[pidType][axisIndex] > -1)
                    fieldValue = frame[flightLog->mainFieldIndexes.pid[pidType][axisIndex]];
                else
                    fieldValue = 0;
            } else
                fieldValue = 0;

            snprintf(fieldLabel, sizeof(fieldLabel), "%d", fieldValue);

            switch (pidType) {
                case PID_P - 1:
                    cairo_set_source_rgb(
                        cr,
                        fieldMeta.gyroColors[axisIndex].r,
                        fieldMeta.gyroColors[axisIndex].g,
                        fieldMeta.gyroColors[axisIndex].b
                    );
                break;
                case PID_TOTAL:
                    cairo_set_source_rgb(
                        cr,
                        WHITE.r,
                        WHITE.g,
                        WHITE.b
                    );
                break;
                default:
                    cairo_set_source_rgb(
                        cr,
                        fieldMeta.PIDAxisColors[pidType][axisIndex].r,
                        fieldMeta.PIDAxisColors[pidType][axisIndex].g,
                        fieldMeta.PIDAxisColors[pidType][axisIndex].b
                    );
                break;
            }

            cairo_move_to (
                cr,
                FIRST_COL_LEFT + (pidType + 1) * HORZ_SPACING,
                FIRST_ROW_TOP + axisIndex * VERT_SPACING + fontExtent.height
            );
            cairo_show_text (cr, fieldLabel);
        }
    }

    cairo_restore(cr);
}

//Draw an origin line for a graph (at the origin and spanning the window)
void drawAxisLine(cairo_t *cr)
{
    cairo_save(cr);

    cairo_set_source_rgba(cr, 1, 1, 1, 0.5);

    cairo_set_dash(cr, 0, 0, 0);
    cairo_set_line_width(cr, 1);
    cairo_move_to(cr, 0, 0);
    cairo_line_to(cr, options.imageWidth, 0);
    cairo_stroke(cr);

    cairo_restore(cr);
}

void drawAxisLabel(cairo_t *cr, const char *axisLabel)
{
    cairo_text_extents_t extent;

    cairo_set_font_size(cr, FONTSIZE_AXIS_LABEL);
    cairo_set_source_rgba(cr, 1, 1, 1, 0.9);

    cairo_text_extents(cr, axisLabel, &extent);
    cairo_move_to(cr, options.imageWidth - 8 - extent.width, -8);
    cairo_show_text(cr, axisLabel);
}

void drawFrameLabel(cairo_t *cr, uint32_t frameIndex, uint32_t frameTimeMsec)
{
    char frameNumberBuf[16];
    cairo_text_extents_t extentFrameNumber, extentFrameTime;

    snprintf(frameNumberBuf, sizeof(frameNumberBuf), "#%07u", frameIndex);

    cairo_set_font_size(cr, FONTSIZE_FRAME_LABEL);
    cairo_set_source_rgba(cr, 1, 1, 1, 0.65);

    cairo_text_extents(cr, "#0000000", &extentFrameNumber);

    cairo_move_to(cr, options.imageWidth - extentFrameNumber.width - 8, options.imageHeight - 8);
    cairo_show_text(cr, frameNumberBuf);

    int frameSec, frameMins;

    frameSec = frameTimeMsec / 1000;
    frameTimeMsec %= 1000;

    frameMins = frameSec / 60;
    frameSec %= 60;

    snprintf(frameNumberBuf, sizeof(frameNumberBuf), "%02d:%02d.%03d", frameMins, frameSec, frameTimeMsec);

    cairo_text_extents(cr, "00:00.000", &extentFrameTime);

    cairo_move_to(cr, options.imageWidth - extentFrameTime.width - 8, options.imageHeight - 8 - extentFrameNumber.height - 8);
    cairo_show_text(cr, frameNumberBuf);
}

void drawAccelerometerData(cairo_t *cr, int64_t *frame)
{
    int16_t accSmooth[3];
    attitude_t attitude;
    t_fp_vector acceleration;
    double magnitude;
    static double lastAccel = 0, lastVoltage = 0, lastCurrent = 0;
    static int lastAlt = 0;
    cairo_text_extents_t extent;

    char labelBuf[32];

    cairo_set_font_size(cr, FONTSIZE_FRAME_LABEL);
    cairo_set_source_rgba(cr, 1, 1, 1, 0.65);

    cairo_text_extents(cr, "Acceleration 0.0G", &extent);

    if (flightLog->sysConfig.acc_1G && fieldMeta.hasAccs) {
        for (int axis = 0; axis < 3; axis++)
            accSmooth[axis] = frame[flightLog->mainFieldIndexes.accSmooth[axis]];

        attitude.roll = intToFloat(frame[fieldMeta.roll]);
        attitude.pitch = intToFloat(frame[fieldMeta.pitch]);
        attitude.heading = intToFloat(frame[fieldMeta.heading]);

        //Need to calculate acc in earth frame in order to subtract the 1G of gravity from the result
        acceleration = calculateAccelerationInEarthFrame(accSmooth, &attitude, flightLog->sysConfig.acc_1G);

        //Now that G has been subtracted, work out the length of the acceleration vector
        acceleration.V.X /= flightLog->sysConfig.acc_1G;
        acceleration.V.Y /= flightLog->sysConfig.acc_1G;
        acceleration.V.Z /= flightLog->sysConfig.acc_1G;

        magnitude = sqrt(acceleration.V.X * acceleration.V.X + acceleration.V.Y * acceleration.V.Y + acceleration.V.Z * acceleration.V.Z);

        //Weighted moving average with the recent history to smooth out noise
        lastAccel = (lastAccel * 2 + magnitude) / 3;

        cairo_move_to(cr, X_POS_LABEL, options.imageHeight - 8);
        cairo_show_text(cr, "Accel.");

        snprintf(labelBuf, sizeof(labelBuf), "%.2f G", lastAccel);

        cairo_move_to(cr, X_POS_VALUE, options.imageHeight - 8);
        cairo_show_text(cr, labelBuf);
    }

    if (flightLog->mainFieldIndexes.vbatLatest > -1) {
        lastVoltage = (lastVoltage * 2 + flightLogVbatADCToMillivolts(flightLog, frame[flightLog->mainFieldIndexes.vbatLatest]) / (1000.0 * fieldMeta.numCells)) / 3;

        cairo_move_to(cr, X_POS_LABEL, options.imageHeight - 8 - (extent.height + 8));
        cairo_show_text(cr, "Batt. cell");

        snprintf(labelBuf, sizeof(labelBuf), "%.2f V", lastVoltage);

        cairo_move_to(cr, X_POS_VALUE, options.imageHeight - 8 - (extent.height + 8));
        cairo_show_text(cr, labelBuf);
    }

    if (flightLog->mainFieldIndexes.BaroAlt > -1) {
        lastAlt = (lastAlt * 2 + frame[flightLog->mainFieldIndexes.BaroAlt]) / 3;

        cairo_move_to(cr, X_POS_LABEL, options.imageHeight - 8 - (extent.height + 8) * 2);
        cairo_show_text(cr, "Altitude");

        snprintf(labelBuf, sizeof(labelBuf), "%.1f m", lastAlt / 100.0);

        cairo_move_to(cr, X_POS_VALUE, options.imageHeight - 8 - (extent.height + 8) * 2);
        cairo_show_text(cr, labelBuf);
    }

    if (flightLog->mainFieldIndexes.amperageLatest > -1) {
        lastCurrent = (lastCurrent * 2 + flightLogAmperageADCToMilliamps(flightLog, frame[flightLog->mainFieldIndexes.amperageLatest]) / 1000.0) / 3;
        cairo_move_to(cr, X_POS_LABEL, options.imageHeight - 8 - (extent.height + 8) * 3);
        cairo_show_text(cr, "Current");

        snprintf(labelBuf, sizeof(labelBuf), "%.2f A", lastCurrent);
        cairo_move_to(cr, X_POS_VALUE, options.imageHeight - 8 - (extent.height + 8) * 3);
        cairo_show_text(cr, labelBuf);

        cairo_move_to(cr, X_POS_VALUE + 140, options.imageHeight - 8 - (extent.height + 8) * 3);
        cairo_show_text(cr, "Total");

        snprintf(labelBuf, sizeof(labelBuf), "%" PRId64 " mAh", frame[fieldMeta.cumulativeCurrent]);
        cairo_move_to(cr, X_POS_VALUE + 220, options.imageHeight - 8 - (extent.height + 8) * 3);
        cairo_show_text(cr, labelBuf);

	if (options.rawAmperage) {
          cairo_move_to(cr, X_POS_VALUE + 400, options.imageHeight - 8 - (extent.height + 8) * 3);
          cairo_show_text(cr, "ADC");

	  snprintf(labelBuf, sizeof(labelBuf), "%" PRId64, frame[flightLog->mainFieldIndexes.amperageLatest]);
          cairo_move_to(cr, X_POS_VALUE + 470, options.imageHeight - 8 - (extent.height + 8) * 3);
          cairo_show_text(cr, labelBuf);
        }
    }
}

void* pngRenderThread(void *arg)
{
    char filename[256];
    pngRenderingTask_t *task = (pngRenderingTask_t *) arg;

    snprintf(filename, sizeof(filename), "%s.%02d.%06d.png", options.outputPrefix, task->outputLogIndex + 1, task->outputFrameIndex);
    cairo_surface_write_to_png (task->surface, filename);
    cairo_surface_destroy (task->surface);

    //Release our slot in the rendering pool, we're done
    semaphore_signal(&pngRenderingSem);

    free(task);

    return 0;
}

/**
 * PNG encoding is so slow and so easily run in parallel, so save the frames using this function
 * (which'll use extra threads to do the work). Be sure to call waitForFramesToSave() before
 * the program ends.
 */
void saveSurfaceAsync(cairo_surface_t *surface, int logIndex, int outputFrameIndex)
{
    if (!pngRenderingSemCreated) {
        semaphore_create(&pngRenderingSem, options.threads);
        pngRenderingSemCreated = true;
    }

    pngRenderingTask_t *task = (pngRenderingTask_t*) malloc(sizeof(*task));

    task->surface = surface;
    task->outputLogIndex = logIndex;
    task->outputFrameIndex = outputFrameIndex;

    // Reserve a slot in the rendering pool...
    semaphore_wait(&pngRenderingSem);

    thread_create_detached(pngRenderThread, task);
}

void waitForFramesToSave()
{
    int i;

    if (pngRenderingSemCreated) {
        for (i = 0; i < options.threads; i++) {
            semaphore_wait(&pngRenderingSem);
        }
    }
}

void renderAnimation(uint32_t startFrame, uint32_t endFrame)
{
    //Change how much data is displayed at one time
    const int windowWidthMicros = 1000 * 1000;

    //Bring the current time into the center of the plot
    const int startXTimeOffset = windowWidthMicros / 2;

    int i;

    int64_t logStartTime = flightLog->stats.field[FLIGHT_LOG_FIELD_INDEX_TIME].min;
    int64_t logEndTime = flightLog->stats.field[FLIGHT_LOG_FIELD_INDEX_TIME].max;
    int64_t logDurationMicro;

    uint32_t outputFrames;

    int64_t frameValues[FLIGHT_LOG_MAX_FIELDS];
    uint64_t lastCenterTime;
    int64_t frameTime;

    FT_Face ft_face;
    cairo_font_face_t *cairo_face;

    struct craftDrawingParameters_t craftParameters;

    //If sync beep time looks reasonable, start the log there instead of at the first frame
    if (abs((int) ((int64_t)syncBeepTime - logStartTime)) < 1000000) //Expected to be well within 1 second of the start
        logStartTime = syncBeepTime;

    logDurationMicro = logEndTime - logStartTime;

    if (endFrame == (uint32_t) -1) {
        endFrame = (uint32_t) ((logDurationMicro * options.fps + (1000000 - 1)) / 1000000);
    }
    outputFrames = endFrame - startFrame;

    if (FT_New_Memory_Face(freetypeLibrary, (const FT_Byte*)SourceSansPro_Regular_otf, SourceSansPro_Regular_otf_len, 0, &ft_face)) {
        fprintf(stderr, "Failed to load font file\n");
        exit(-1);
    }
    cairo_face = cairo_ft_font_face_create_for_ft_face(ft_face, 0);

    decideCraftParameters(&craftParameters, options.imageWidth, options.imageHeight);

    //Exaggerate values around the origin and compress values near the edges:
    pitchStickCurve = expoCurveCreate(0, 0.700, 500 * (flightLog->sysConfig.rcRate ? flightLog->sysConfig.rcRate : 100) / 100, 1.0, 10);

    gyroCurve = expoCurveCreate(0, 0.2, 9.0e-6 / flightLog->sysConfig.gyroScale, 1.0, 10);
    accCurve = expoCurveCreate(0, 0.7, 5000, 1.0, 10);
    pidCurve = expoCurveCreate(0, 0.7, 500, 1.0, 10);

    motorCurve = expoCurveCreate(-(flightLog->sysConfig.motorOutputHigh + flightLog->sysConfig.motorOutputLow) / 2, 1.0,
            (flightLog->sysConfig.motorOutputHigh - flightLog->sysConfig.motorOutputLow) / 2, 1.0, 2);

    // Default Servo range is [1020...2000] but we'll just use [1000...2000] for simplicity
    servoCurve = expoCurveCreate(-1500, 1.0, 1000, 1.0, 2);

    int durationSecs = (outputFrames + (options.fps - 1)) / (options.fps);
    int durationMins = durationSecs / 60;
    durationSecs %= 60;

    fprintf(stderr, "%d frames to be rendered at %d FPS [%d:%02d]\n", outputFrames, options.fps, durationMins, durationSecs);
    fprintf(stderr, "\n");

    for (uint32_t outputFrameIndex = startFrame; outputFrameIndex < endFrame; outputFrameIndex++) {
        int64_t windowCenterTime = logStartTime + ((int64_t) outputFrameIndex * 1000000) / options.fps;
        int64_t windowStartTime = windowCenterTime - startXTimeOffset;
        int64_t windowEndTime = windowStartTime + windowWidthMicros;

        cairo_surface_t *surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, options.imageWidth, options.imageHeight);
        cairo_t *cr = cairo_create(surface);

        // Find the frame just to the left of the first pixel so we can start drawing lines from there
        int firstFrameIndex = datapointsFindFrameAtTime(points, windowStartTime - 1);

        if (firstFrameIndex == -1) {
            firstFrameIndex = 0;
        }

        cairo_set_font_face(cr, cairo_face);

        //Plot the upper motor graph
        if (options.plotMotors) {
            int motorGraphHeight = (int) (options.imageHeight * (options.plotPids ? 0.15 : 0.20));

            cairo_save(cr);
            {
                if (options.plotPids) {
                    //Move up a little bit to make room for the pid graphs
                    cairo_translate(cr, 0, options.imageHeight * 0.15);
                } else {
                    cairo_translate(cr, 0, options.imageHeight * 0.25);
                }

                drawAxisLine(cr);

                cairo_set_line_width(cr, 2.5);

                for (i = 0; i < fieldMeta.numMotors; i++) {
                    plotLine(cr, fieldMeta.motorColors[i], windowStartTime, windowEndTime, firstFrameIndex,
                            flightLog->mainFieldIndexes.motor[i], motorCurve, motorGraphHeight);
                }

                if (fieldMeta.numServos) {
                    for (i = 0; i < MAX_SERVOS; i++) {
                        if (flightLog->mainFieldIndexes.servo[i] > -1) {
                            plotLine(cr, fieldMeta.servoColors[i], windowStartTime, windowEndTime, firstFrameIndex,
                                flightLog->mainFieldIndexes.servo[i], motorCurve, motorGraphHeight);
                        }
                    }
                }

                drawAxisLabel(cr, "Motors");
            }
            cairo_restore(cr);
        }

        //Plot the lower PID graphs
        cairo_save(cr);
        {
            if (options.plotPids) {
                //Plot three axes as different graphs
                cairo_translate(cr, 0, options.imageHeight * 0.60);
                for (int axis = 0; axis < 3; axis++) {
                    cairo_save(cr);

                    cairo_translate(cr, 0, options.imageHeight * 0.2 * (axis - 1));

                    drawAxisLine(cr);

                    for (int pidType = PID_D; pidType >= PID_P; pidType--) {
                        if (flightLog->mainFieldIndexes.pid[pidType][axis] > -1) {
                            switch (pidType) {
                                case PID_P:
                                    cairo_set_line_width(cr, 2);
                                break;
                                case PID_I:
                                    cairo_set_dash(cr, DASHED_LINE, DASHED_LINE_NUM_POINTS, 0);
                                    cairo_set_line_width(cr, 2);
                                break;
                                case PID_D:
                                    cairo_set_dash(cr, DOTTED_LINE, DOTTED_LINE_NUM_POINTS, 0);
                                    cairo_set_line_width(cr, 2);
                            }

                            plotLine(cr, fieldMeta.PIDAxisColors[pidType][axis], windowStartTime, windowEndTime, firstFrameIndex,
                                    flightLog->mainFieldIndexes.pid[pidType][axis], pidCurve, (int) (options.imageHeight * 0.15));

                            cairo_set_dash(cr, 0, 0, 0);
                        }
                    }

                    if (options.plotGyros) {
                        cairo_set_line_width(cr, 3);

                        plotLine(cr, fieldMeta.gyroColors[axis], windowStartTime, windowEndTime, firstFrameIndex,
                            flightLog->mainFieldIndexes.gyroADC[axis], gyroCurve, (int) (options.imageHeight * 0.15));
                    }

                    const char *axisLabel;
                    if (options.plotGyros) {
                        switch (axis) {
                            case 0:
                                axisLabel = "Gyro + PID roll";
                            break;
                            case 1:
                                axisLabel = "Gyro + PID pitch";
                            break;
                            case 2:
                                axisLabel = "Gyro + PID yaw";
                            break;
                            default:
                                axisLabel = "Unknown";
                        }
                    } else {
                        switch (axis) {
                            case 0:
                                axisLabel = "Roll PIDs";
                            break;
                            case 1:
                                axisLabel = "Pitch PIDs";
                            break;
                            case 2:
                                axisLabel = "Yaw PIDs";
                            break;
                            default:
                                axisLabel = "Unknown";
                        }
                    }

                    drawAxisLabel(cr, axisLabel);

                    cairo_restore(cr);
                }
            } else if (options.plotGyros) {
                //Plot three gyro axes on one graph
                cairo_translate(cr, 0, options.imageHeight * 0.70);

                drawAxisLine(cr);

                for (int axis = 0; axis < 3; axis++) {
                    plotLine(cr, fieldMeta.gyroColors[axis], windowStartTime, windowEndTime, firstFrameIndex,
                            flightLog->mainFieldIndexes.gyroADC[axis], gyroCurve, (int) (options.imageHeight * 0.25));
                }

                drawAxisLabel(cr, "Gyro");
            }
        }
        cairo_restore(cr);

        //Draw a bar highlighting the current time if we are drawing any graphs
        if (options.plotGyros || options.plotMotors || options.plotPids || options.plotPidSum) {
            double centerX = options.imageWidth / 2.0;

            cairo_set_source_rgba(cr, 1, 0.25, 0.25, 0.2);
            cairo_set_line_width(cr, 20);

            cairo_move_to(cr, centerX, 0);
            cairo_line_to(cr, centerX, options.imageHeight);
            cairo_stroke(cr);
        }

        int centerFrameIndex = datapointsFindFrameAtTime(points, windowCenterTime);

        //Draw the command stick positions from the centered frame
        if (datapointsGetFrameAtIndex(points, centerFrameIndex, &frameTime, frameValues)) {
            if (options.drawSticks) {
                cairo_save(cr);
                {

                    if(options.sticksTop != 0 && options.sticksRight != 0) {
                      cairo_translate(cr, options.imageWidth - options.sticksRight, options.sticksTop);
                    } else if(options.sticksRight != 0) {
                      cairo_translate(cr, options.imageWidth - options.sticksRight, 0.20 * options.imageHeight);
                    } else if(options.sticksTop != 0) {
                      cairo_translate(cr, 0.75 * options.imageWidth, options.sticksTop);
                    } else {
                      cairo_translate(cr, 0.75 * options.imageWidth, 0.20 * options.imageHeight);
                    }

                    drawCommandSticks(frameValues, options.imageWidth, options.imageHeight, cr);
                }
                cairo_restore(cr);
            }

            if (options.drawPidTable) {
                cairo_save(cr);
                {
                    cairo_translate(cr, 0.25 * options.imageWidth, 0.75 * options.imageHeight);
                    drawPIDTable(cr, frameValues);
                }
                cairo_restore(cr);
            }

            if (options.drawCraft) {
                cairo_save(cr);
                {
                    if(options.craftTop != 0 && options.craftRight != 0) {
                      cairo_translate(cr, options.imageWidth - options.craftRight, options.craftTop);
                    } else if(options.craftRight != 0) {
                      cairo_translate(cr, options.imageWidth - options.craftRight, 0.20 * options.imageHeight);
                    } else if(options.craftTop != 0) {
                      cairo_translate(cr, 0.75 * options.imageWidth, options.craftTop);
                    } else {
                      cairo_translate(cr, 0.75 * options.imageWidth, 0.20 * options.imageHeight);
                    }

                    drawCraft(cr, frameValues, outputFrameIndex > 0 ? windowCenterTime - lastCenterTime : 0, &craftParameters);
                }
                cairo_restore(cr);
            }

            if (options.drawAcc) {
              drawAccelerometerData(cr, frameValues);
            }

            if (options.drawTime)
                drawFrameLabel(cr, frameValues[FLIGHT_LOG_FIELD_INDEX_ITERATION], (uint32_t) ((windowCenterTime - flightLog->stats.field[FLIGHT_LOG_FIELD_INDEX_TIME].min) / 1000));
        }

        // Draw a synchronisation line
        if (syncBeepTime >= windowStartTime && syncBeepTime < windowEndTime) {
            double lineX = (double) ((int64_t) options.imageWidth * (syncBeepTime - windowStartTime) / windowWidthMicros);

            cairo_set_source_rgba(cr, 0.25, 0.25, 1, 0.2);
            cairo_set_line_width(cr, 20);

            cairo_move_to(cr, lineX, 0);
            cairo_line_to(cr, lineX, options.imageHeight);
            cairo_stroke(cr);
        }

        cairo_destroy(cr);

        lastCenterTime = windowCenterTime;

        saveSurfaceAsync(surface, selectedLogIndex, outputFrameIndex);

        uint32_t frameWrittenCount = outputFrameIndex - startFrame + 1;
        if (frameWrittenCount % 500 == 0 || frameWrittenCount == outputFrames) {
            fprintf(stderr, "Rendered %d frames (%.1f%%)%s\n",
                frameWrittenCount, (double)frameWrittenCount / outputFrames * 100,
                frameWrittenCount < outputFrames ? "..." : ".");
        }
    }

    waitForFramesToSave();
}

void printUsage(const char *argv0)
{
    fprintf(stderr,
        "Blackbox flight log renderer by Nicholas Sherlock ("
#ifdef BLACKBOX_VERSION
            "v" STR(BLACKBOX_VERSION) ", "
#endif
            __DATE__ " " __TIME__ ")\n\n"
        "Usage:\n"
        "     %s [options] <logfilename.txt>\n\n"
        "Options:\n"
        "   --help                 This page\n"
        "   --index <num>          Choose which log from the file should be rendered\n"
        "   --width <px>           Choose the width of the image (default %d)\n"
        "   --height <px>          Choose the height of the image (default %d)\n"
        "   --fps                  FPS of the resulting video (default %d)\n"
        "   --threads              Number of threads to use to render frames (default %d)\n"
        "   --prefix <filename>    Set the prefix of the output frame filenames\n"
        "   --start <x:xx>         Begin the log at this time offset (default 0:00)\n"
        "   --end <x:xx>           End the log at this time offset\n"
        "   --[no-]draw-pid-table  Show table with PIDs and gyros (default on)\n"
        "   --[no-]draw-craft      Show craft drawing (default on)\n"
        "   --[no-]draw-sticks     Show RC command sticks (default on)\n"
        "   --[no-]draw-time       Show frame number and time in bottom right (default on)\n"
        "   --[no-]draw-acc        Show accelerometer data and amperage in bottom left (default on)\n"
        "   --[no-]plot-motor      Draw motors on the upper graph (default on)\n"
        "   --[no-]plot-pid        Draw PIDs on the lower graph (default off)\n"
        "   --[no-]plot-gyro       Draw gyroscopes on the lower graph (default on)\n"
        "   --sticks-top <px>      Offset the stick overlay from the top (default off)\n"
        "   --sticks-right <px>    Offset the stick overlay from the right (default off)\n"
        "   --sticks-width <px>    Size of the stick area (default off)\n"
        "   --sticks-radius <px>   Radius of the sticks (default relative to image size)\n"
        "   --craft-top <px>       Offset the craft overlay from the top (default off)\n"
        "   --craft-right <px>     Offset the craft overlay from the right (default off)\n"
        "   --craft-width <px>     Size of the craft area (default off)\n"
        "   --smoothing-pid <n>    Smoothing window for the PIDs (default %d)\n"
        "   --smoothing-gyro <n>   Smoothing window for the gyroscopes (default %d)\n"
        "   --smoothing-motor <n>  Smoothing window for the motors (default %d)\n"
        "   --unit-gyro <raw|degree>  Unit for the gyro values in the table (default %s)\n"
        "   --prop-style <name>    Style of propeller display (pie/blades, default %s)\n"
        "   --gapless              Fill in gaps in the log with straight lines\n"
        "   --raw-amperage         Print the current sensor ADC value along with computed amperage\n"
        "   --sticks-text-color    Set the RGBA text color (default 1.0,1.0,1.0,1.0)\n"
        "   --sticks-color         Set the RGBA sticks color (default 1.0,0.4,0.4,1.0)\n"
        "   --sticks-area-color    Set the RGBA sticks area color (default 0.3,0.3,0.3,0.8)\n"
        "   --sticks-cross-color   Set the RGBA sticks crosshair color (default 0.75,0.75,0.75,0.5)\n"
        "   --sticks-trail-length <px> Length of the stick trails (default %d)\n"
        "   --sticks-trail-color   Set the RGBA stick trail color (default 1.0,1.0,1.0,1.0)\n"
        "\n", argv0, defaultOptions.imageWidth, defaultOptions.imageHeight, defaultOptions.fps, defaultOptions.threads,
            defaultOptions.pidSmoothing, defaultOptions.gyroSmoothing, defaultOptions.motorSmoothing,
            UNIT_NAME[defaultOptions.gyroUnit], PROP_STYLE_NAME[defaultOptions.propStyle], defaultOptions.stickTrailLength
    );
}

/**
 * Parse a time formatted as "%d" (secs) or "%d:%d" (mins:secs) into a count of seconds and
 * store it into frameTime.
 *
 * If the text could not be parsed, false is returned and frameTime is not modified.
 */
bool parseFrameTime(const char *text, uint32_t *frameTime)
{
    const char *cur, *colon = 0, *end;

    for (cur = text; *cur; cur++) {
        if (*cur == ':') {
            if (colon)
                return false;
            else
                colon = cur;
        } else if (!(*cur >= '0' && *cur <= '9'))
            return false;
    }

    end = cur;

    //Check that there are digits before and after the colon and the string wasn't empty...
    if (end == text || (colon && (colon == text || colon == end - 1)))
        return false;

    if (colon) {
        int mins = atoi(text);
        int secs = atoi(colon + 1);

        *frameTime = mins * 60 + secs;
    } else {
        *frameTime = atoi(text);
    }

    return true;
}

bool parseTextColor(const char *text, colorAlpha_t *color) {
  int counter = 0;
  const char *cur;

  color->r = atof(text);
  for(cur = text; *cur; cur++) {
    if(*cur == ',') {
      switch(counter) {
        case 0: {
          color->g = atof(cur + 1);
        } break;
        case 1: {
          color->b = atof(cur + 1);
        } break;
        case 2: {
          color->a = atof(cur + 1);
        } break;
      }

      counter++;
    }
  }

  if (counter != 3) {
    return false;
  }

  return true;
}

Unit parseUnit(const char *s)
{
    if (strcmp(s, "degree") == 0 || strcmp(s, "degrees") == 0)
        return UNIT_DEGREES_PER_SEC;
    return UNIT_RAW;
}

void parseCommandlineOptions(int argc, char **argv)
{
    int option_index = 0;
    int c;
    enum {
        SETTING_INDEX = 1,
        SETTING_WIDTH,
        SETTING_HEIGHT,
        SETTING_FPS,
        SETTING_PREFIX,
        SETTING_START,
        SETTING_END,
        SETTING_SMOOTHING_PID,
        SETTING_SMOOTHING_GYRO,
        SETTING_SMOOTHING_MOTOR,
        SETTING_UNIT_GYRO,
        SETTING_PROP_STYLE,
        SETTING_THREADS,
        SETTING_STICKS_TOP,
        SETTING_STICKS_RIGHT,
        SETTING_STICKS_WIDTH,
        SETTING_STICKS_TEXT_COLOR,
        SETTING_STICK_COLOR,
        SETTING_STICK_AREA_COLOR,
        SETTING_STICK_CROSSHAIR_COLOR,
        SETTING_STICK_TRAIL_LENGTH,
        SETTING_STICK_TRAIL_COLOR,
        SETTING_CRAFT_TOP,
        SETTING_CRAFT_RIGHT,
        SETTING_CRAFT_WIDTH,
        SETTING_STICK_RADIUS,
        SETTING_STICK_TRAIL_RADIUS,
    };

    memcpy(&options, &defaultOptions, sizeof(options));

    while (1)
    {
        static struct option long_options[] = {
            {"help", no_argument, &options.help, 1},
            {"index", required_argument, 0, SETTING_INDEX},
            {"width", required_argument, 0, SETTING_WIDTH},
            {"height", required_argument, 0, SETTING_HEIGHT},
            {"fps", required_argument, 0, SETTING_FPS},
            {"prefix", required_argument, 0, SETTING_PREFIX},
            {"start", required_argument, 0, SETTING_START},
            {"end", required_argument, 0, SETTING_END},
            {"plot-pid", no_argument, &options.plotPids, 1},
            {"plot-gyro", no_argument, &options.plotGyros, 1},
            {"plot-motor", no_argument, &options.plotMotors, 1},
            {"no-plot-pid", no_argument, &options.plotPids, 0},
            {"no-plot-gyro", no_argument, &options.plotGyros, 0},
            {"no-plot-motor", no_argument, &options.plotMotors, 0},
            {"draw-pid-table", no_argument, &options.drawPidTable, 1},
            {"draw-craft", no_argument, &options.drawCraft, 1},
            {"draw-sticks", no_argument, &options.drawSticks, 1},
            {"draw-time", no_argument, &options.drawTime, 1},
            {"draw-acc", no_argument, &options.drawAcc, 1},
            {"no-draw-pid-table", no_argument, &options.drawPidTable, 0},
            {"no-draw-craft", no_argument, &options.drawCraft, 0},
            {"no-draw-sticks", no_argument, &options.drawSticks, 0},
            {"no-draw-time", no_argument, &options.drawTime, 0},
            {"no-draw-acc", no_argument, &options.drawAcc, 0},
            {"smoothing-pid", required_argument, 0, SETTING_SMOOTHING_PID},
            {"smoothing-gyro", required_argument, 0, SETTING_SMOOTHING_GYRO},
            {"smoothing-motor", required_argument, 0, SETTING_SMOOTHING_MOTOR},
            {"unit-gyro", required_argument, 0, SETTING_UNIT_GYRO},
            {"prop-style", required_argument, 0, SETTING_PROP_STYLE},
            {"threads", required_argument, 0, SETTING_THREADS},
            {"gapless", no_argument, &options.gapless, 1},
            {"raw-amperage", no_argument, &options.rawAmperage, 1},
            {"sticks-top", required_argument, 0, SETTING_STICKS_TOP},
            {"sticks-right", required_argument, 0, SETTING_STICKS_RIGHT},
            {"sticks-width", required_argument, 0, SETTING_STICKS_WIDTH},
            {"sticks-text-color", required_argument, 0, SETTING_STICKS_TEXT_COLOR},
            {"sticks-color", required_argument, 0, SETTING_STICK_COLOR},
            {"sticks-area-color", required_argument, 0, SETTING_STICK_AREA_COLOR},
            {"sticks-cross-color", required_argument, 0, SETTING_STICK_CROSSHAIR_COLOR},
            {"sticks-trail-length", required_argument, 0, SETTING_STICK_TRAIL_LENGTH},
            {"sticks-trail-color", required_argument, 0, SETTING_STICK_TRAIL_COLOR},
            {"craft-top", required_argument, 0, SETTING_CRAFT_TOP},
            {"craft-right", required_argument, 0, SETTING_CRAFT_RIGHT},
            {"craft-width", required_argument, 0, SETTING_CRAFT_WIDTH},
            {"sticks-radius", required_argument, 0, SETTING_STICK_RADIUS},
            {"sticks-trail-radius", required_argument, 0, SETTING_STICK_TRAIL_RADIUS},
            {0, 0, 0, 0}
        };

        opterr = 0;

        c = getopt_long (argc, argv, ":", long_options, &option_index);

        if (c == -1)
            break;

        switch (c) {
            case SETTING_START:
                if (!parseFrameTime(optarg, &options.timeStart))  {
                    fprintf(stderr, "Bad --start time value\n");
                    exit(-1);
                }
            break;
            case SETTING_END:
                if (!parseFrameTime(optarg, &options.timeEnd))  {
                    fprintf(stderr, "Bad --end time value\n");
                    exit(-1);
                }
            break;
            case SETTING_STICKS_TEXT_COLOR:
                if (!parseTextColor(optarg, &options.sticksTextColor))  {
                    fprintf(stderr, "Bad --sticks-text-color color value\n");
                    exit(-1);
                }
            break;
            case SETTING_STICK_COLOR:
                if (!parseTextColor(optarg, &options.stickColor))  {
                    fprintf(stderr, "Bad --sticks-color color value\n");
                    exit(-1);
                }
            break;
            case SETTING_STICK_AREA_COLOR:
                if (!parseTextColor(optarg, &options.stickAreaColor))  {
                    fprintf(stderr, "Bad --sticks-area-color color value\n");
                    exit(-1);
                }
            break;
            case SETTING_STICK_CROSSHAIR_COLOR:
                if (!parseTextColor(optarg, &options.crosshairColor))  {
                    fprintf(stderr, "Bad --sticks-cross-color color value\n");
                    exit(-1);
                }
            break;
            case SETTING_STICK_TRAIL_COLOR:
                if (!parseTextColor(optarg, &options.stickTrailColor))  {
                    fprintf(stderr, "Bad --sticks-trail-color color value\n");
                    exit(-1);
                }
            break;
            case SETTING_WIDTH:
                options.imageWidth = atoi(optarg);
            break;
            case SETTING_HEIGHT:
                options.imageHeight = atoi(optarg);
            break;
            case SETTING_FPS:
                options.fps = atoi(optarg);
            break;
            case SETTING_PREFIX:
                options.outputPrefix = optarg;
            break;
            case SETTING_SMOOTHING_PID:
                options.pidSmoothing = atoi(optarg);
            break;
            case SETTING_SMOOTHING_GYRO:
                options.gyroSmoothing = atoi(optarg);
            break;
            case SETTING_SMOOTHING_MOTOR:
                options.motorSmoothing = atoi(optarg);
            break;
            case SETTING_UNIT_GYRO:
                options.gyroUnit = parseUnit(optarg);
            break;
            case SETTING_THREADS:
                options.threads = atoi(optarg);
                if (options.threads < 1) {
                    options.threads = 1;
                }
            break;
            case SETTING_INDEX:
                options.logNumber = atoi(optarg);
            break;
            case SETTING_PROP_STYLE:
                if (strcmp(optarg, "pie") == 0) {
                    options.propStyle = PROP_STYLE_PIE_CHART;
                } else {
                    options.propStyle = PROP_STYLE_BLADES;
                }
            break;
            case SETTING_STICKS_TOP:
                options.sticksTop = atoi(optarg);
            break;
            case SETTING_STICKS_RIGHT:
                options.sticksRight = atoi(optarg);
            break;
            case SETTING_STICKS_WIDTH:
                options.sticksWidth = atoi(optarg);
            break;
            case SETTING_STICK_RADIUS:
                options.stickRadius = atoi(optarg);
            break;
            case SETTING_STICK_TRAIL_RADIUS:
                options.stickTrailRadius = atoi(optarg);
            break;
            case SETTING_STICK_TRAIL_LENGTH:
                options.stickTrailLength = atoi(optarg);
            break;
            case SETTING_CRAFT_WIDTH:
                options.craftWidth = atoi(optarg);
            break;
            case SETTING_CRAFT_TOP:
                options.craftTop = atoi(optarg);
            break;
            case SETTING_CRAFT_RIGHT:
                 options.craftRight = atoi(optarg);
            break;
            case '\0':
                //Longopt which has set a flag
            break;
            case ':':
                fprintf(stderr, "%s: option '%s' requires an argument\n", argv[0], argv[optind-1]);
                exit(-1);
            break;
            default:
                if (optopt == 0)
                    fprintf(stderr, "%s: option '%s' is invalid\n", argv[0], argv[optind-1]);
                else
                    fprintf(stderr, "%s: option '-%c' is invalid\n", argv[0], optopt);

                exit(-1);
            break;
        }
    }

    if (optind < argc) {
        options.filename = argv[optind];
    }
}

static void applySmoothing() {
    if (options.gyroSmoothing && fieldMeta.hasGyros) {
        for (int axis = 0; axis < 3; axis++)
            datapointsSmoothField(points, flightLog->mainFieldIndexes.gyroADC[axis], options.gyroSmoothing);
    }

    if (options.pidSmoothing && fieldMeta.hasPIDs) {
        for (int pid = PID_P; pid <= PID_D; pid++)
            for (int axis = 0; axis < 3; axis++)
                if (flightLog->mainFieldIndexes.pid[pid][axis] > -1)
                    datapointsSmoothField(points, flightLog->mainFieldIndexes.pid[pid][axis], options.pidSmoothing);

        //Smooth the synthetic PID sum field too
        for (int axis = 0; axis < 3; axis++)
            datapointsSmoothField(points, fieldMeta.axisPIDSum[axis], options.pidSmoothing);
    }

    if (options.motorSmoothing) {
        for (int motor = 0; motor < fieldMeta.numMotors; motor++)
            datapointsSmoothField(points, flightLog->mainFieldIndexes.motor[motor], options.motorSmoothing);
    }
}

void computeExtraFields(void) {
    int16_t accSmooth[3], gyroADC[3], magADC[3];
    int64_t frameTime, lastFrameTime = 0;
    int32_t frameIndex;
    int64_t frame[FLIGHT_LOG_MAX_FIELDS];
    double cumulativeCurrent = 0.0; // in milliamp-hours
    attitude_t attitude;
    bool calculateAttitude = fieldMeta.hasGyros && fieldMeta.hasAccs && flightLog->sysConfig.acc_1G;

    imuInit();

    for (frameIndex = 0; frameIndex < points->frameCount; frameIndex++) {
        if (datapointsGetFrameAtIndex(points, frameIndex, &frameTime, frame)) {
            if (calculateAttitude) {
                for (int axis = 0; axis < 3; axis++) {
                    accSmooth[axis] = frame[flightLog->mainFieldIndexes.accSmooth[axis]];
                    gyroADC[axis] = frame[flightLog->mainFieldIndexes.gyroADC[axis]];
                }

                if (fieldMeta.hasMagADC) {
                    for (int axis = 0; axis < 3; axis++) {
                        magADC[axis] = frame[flightLog->mainFieldIndexes.magADC[axis]];
                    }
                }

                updateEstimatedAttitude(gyroADC, accSmooth, fieldMeta.hasMagADC ? magADC : 0, (uint32_t) frameTime, flightLog->sysConfig.acc_1G, flightLog->sysConfig.gyroScale, &attitude);

                //Pack those floats into signed ints to store into the datapoints array:
                datapointsSetFieldAtIndex(points, frameIndex, fieldMeta.roll, floatToInt(attitude.roll));
                datapointsSetFieldAtIndex(points, frameIndex, fieldMeta.pitch, floatToInt(attitude.pitch));
                datapointsSetFieldAtIndex(points, frameIndex, fieldMeta.heading, floatToInt(attitude.heading));
            }

            if (fieldMeta.hasPIDs) {
                for (int axis = 0; axis < 3; axis++) {
                    int32_t pidSum = frame[flightLog->mainFieldIndexes.pid[PID_P][axis]] + frame[flightLog->mainFieldIndexes.pid[PID_I][axis]] + frame[flightLog->mainFieldIndexes.pid[PID_D][axis]];

                    datapointsSetFieldAtIndex(points, frameIndex, fieldMeta.axisPIDSum[axis], pidSum);
                }
            }

            if (lastFrameTime != 0 && flightLog->mainFieldIndexes.amperageLatest != -1) {
                /*
                 * Multiply the current measurement against the time since the last loop to get the number of milliamp-hours
                 * consumed (assume that the current usage was constant over that interval)
                 */
                cumulativeCurrent += (frameTime - lastFrameTime) / 1000000.0 / 60 / 60 * flightLogAmperageADCToMilliamps(flightLog, frame[flightLog->mainFieldIndexes.amperageLatest]);

                datapointsSetFieldAtIndex(points, frameIndex, fieldMeta.cumulativeCurrent, round(cumulativeCurrent));
            }
            lastFrameTime = frameTime;
        }
    }
}

int chooseLog(flightLog_t *log)
{
    if (!log || log->logCount == 0) {
        fprintf(stderr, "Couldn't find the header of a flight log in this file, is this the right kind of file?\n");
        return -1;
    }

    //Did the user pick a log to render?
    if (options.logNumber > 0) {
        if (options.logNumber > log->logCount) {
            fprintf(stderr, "Couldn't load log #%d from this file, because there are only %d logs in total.\n", options.logNumber, log->logCount);
            return -1;
        }

        return options.logNumber - 1;
    } else if (log->logCount == 1) {
        // If there's only one log, just parse that
        return 0;
    } else {
        fprintf(stderr, "This file contains multiple flight logs, please choose one with the --index argument:\n\n");

        fprintf(stderr, "Index  Start offset  Size (bytes)\n");
        for (int i = 0; i < log->logCount; i++) {
            fprintf(stderr, "%5d %13d %13d\n", i + 1, (int) (log->logBegin[i] - log->logBegin[0]), (int) (log->logBegin[i + 1] - log->logBegin[i]));
        }

        return -1;
    }
}

int main(int argc, char **argv)
{
    struct stat directoryStat;
    char outputDirectory[256];
    char **fieldNames;
    uint32_t frameStart, frameEnd;
    int fd;

    platform_init();

    parseCommandlineOptions(argc, argv);

    if (options.help || !options.filename) {
        printUsage(argv[0]);
        return -1;
    }

    options.bottomGraphSplitAxes = options.plotPids;

    stickTrails[0] = malloc(options.stickTrailLength * sizeof(struct point_t));
    stickTrails[1] = malloc(options.stickTrailLength * sizeof(struct point_t));

    fd = open(options.filename, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "Failed to open log file '%s': %s\n", options.filename, strerror(errno));
        return -1;
    }

    FT_Init_FreeType(&freetypeLibrary);

    flightLog = flightLogCreate(fd);

    selectedLogIndex = chooseLog(flightLog);

    if (selectedLogIndex == -1)
        return -1;

    //If the user didn't supply an output filename prefix, create our own based on the input filename
    if (!options.outputPrefix) {
        char *fileExtensionPeriod = strrchr(options.filename, '.');
        char *fileSlash = strrchr(options.filename, '/');
        char *logNameStart, *logNameEnd;

        if (strrchr(options.filename, '\\') > fileSlash)
            fileSlash = strrchr(options.filename, '\\');

        if (fileSlash)
            logNameStart = fileSlash + 1;
        else
            logNameStart = options.filename;

        if (fileExtensionPeriod) {
            logNameEnd = fileExtensionPeriod;
        } else {
            logNameEnd = options.filename + strlen(options.filename);
        }

        snprintf(outputDirectory, 256, "%.*s.%02d", (int) (logNameEnd - options.filename), options.filename, selectedLogIndex + 1);

        //Create the output directory if it doesn't exist
        if (stat(outputDirectory, &directoryStat) != 0) {
            directory_create(outputDirectory);
        }

        options.outputPrefix = malloc(256 * sizeof(char));
        snprintf(options.outputPrefix, 256, "%s/%.*s", outputDirectory, (int) (logNameEnd - logNameStart), logNameStart);
    }

    //First check out how many frames we need to store so we can pre-allocate (parsing will update the flightlog stats which contain that info)
    flightLogParse(flightLog, selectedLogIndex, NULL, NULL, NULL, false);

    // Assign field indexes to the fields we'll add
    int newFieldIndex = flightLog->frameDefs['I'].fieldCount, combinedFieldCount;

    fieldMeta.roll = newFieldIndex++;
    fieldMeta.pitch = newFieldIndex++;
    fieldMeta.heading = newFieldIndex++;

    fieldMeta.axisPIDSum[0] = newFieldIndex++;
    fieldMeta.axisPIDSum[1] = newFieldIndex++;
    fieldMeta.axisPIDSum[2] = newFieldIndex++;

    if (flightLog->mainFieldIndexes.amperageLatest > -1) {
        fieldMeta.cumulativeCurrent = newFieldIndex++;
    } else {
        fieldMeta.cumulativeCurrent = -1;
    }

    combinedFieldCount = newFieldIndex;

    // Create a copy of the array of field names so we can add our custom fields to it
    fieldNames = malloc(sizeof(*fieldNames) * combinedFieldCount);

    for (int i = 0; i < flightLog->frameDefs['I'].fieldCount; i++) {
        fieldNames[i] = strdup(flightLog->frameDefs['I'].fieldName[i]);
    }

    // And add our synthetic field names
    fieldNames[fieldMeta.roll] = strdup("roll");
    fieldNames[fieldMeta.pitch] = strdup("pitch");
    fieldNames[fieldMeta.heading] = strdup("heading");
    fieldNames[fieldMeta.axisPIDSum[0]] = strdup("axisPID[0]");
    fieldNames[fieldMeta.axisPIDSum[1]] = strdup("axisPID[1]");
    fieldNames[fieldMeta.axisPIDSum[2]] = strdup("axisPID[2]");

    if (fieldMeta.cumulativeCurrent > -1) {
        fieldNames[fieldMeta.cumulativeCurrent] = strdup("cumulativeCurrent");
    }

    // Create the pre-allocated array of frames that we'll decode into
    points = datapointsCreate(combinedFieldCount, fieldNames, (int) (flightLog->stats.field[FLIGHT_LOG_FIELD_INDEX_ITERATION].max + 1));

    //Now decode the flight log into the points array
    flightLogParse(flightLog, selectedLogIndex, 0, loadFrameIntoPoints, onLogEvent, false);

    updateFieldMetadata();

    computeExtraFields();

    applySmoothing();

    frameStart = options.timeStart * options.fps;

    if (options.timeEnd == 0)
        frameEnd = (uint32_t) -1;
    else {
        frameEnd = options.timeEnd * options.fps;
    }

    if (frameEnd <= frameStart) {
        fprintf(stderr, "Error: Selected end time would make this video zero frames long.\n");
        return -1;
    }

    renderAnimation(frameStart, frameEnd);

    return 0;
}
