#ifndef FSGEOMETRIES_H
#define FSGEOMETRIES_H

typedef float FSFloat; //in cm
typedef unsigned char FSUChar; //in cm
typedef unsigned char FSBool;

/********************************/
/*          FS_COLOR            */
/********************************/

typedef struct _FSColor
{
  FSUChar red;
  FSUChar green;
  FSUChar blue;
} FSColor;

static FSColor FSMakeColor(FSUChar red, FSUChar green, FSUChar blue)
{
    FSColor c;
    c.red = red;
    c.green = green;
    c.blue = blue;
    return c;
}

/********************************/
/*          FS_POINT            */
/********************************/

typedef struct _FSPoint
{
  FSFloat x;
  FSFloat y;
  FSFloat z;
  FSColor color;
} FSPoint;

static FSPoint FSMakePoint(FSFloat x, FSFloat y, FSFloat z)
{
    FSPoint p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

/********************************/
/*          FS_SIZE            */
/********************************/

typedef struct _FSSize
{
  FSFloat width;
  FSFloat height;
  FSFloat depth;
} FSSize;

static FSSize FSMakeSize(FSFloat width, FSFloat height, FSFloat depth)
{
    FSSize s;
    s.width = width;
    s.height = height;
    s.depth = depth;
    return s;
}

/********************************/
/*          FS_MISC             */
/********************************/

typedef enum
{
  FS_DIRECTION_CCW,
  FS_DIRECTION_CW
} FSDirection;

#endif // FSGEOMETRIES_H
