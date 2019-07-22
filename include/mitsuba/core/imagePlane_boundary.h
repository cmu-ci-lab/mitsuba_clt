//
// Created by Jiatian Sun on 2019-06-20.
//

#pragma once

#if !defined(__MITSUBA_IMAGEPLANE_BOUNDARY_H)
#define __MITSUBA_IMAGEPLANE_BOUNDARY_H

MTS_NAMESPACE_BEGIN

struct Boundary{
public:
  enum EBoundaryType{
    EUP = 0X00,
    EDOWN = 0X01,
    ELEFT = 0X02,
    ERIGHT = 0X04,
    EINVALID = 0x08
  };

  static bool isAdjacent(EBoundaryType one, EBoundaryType another){
    switch(another){
      case EUP: return one == ERIGHT || one ==ELEFT;
      case EDOWN: return one == ERIGHT || one == ELEFT;
      case ELEFT: return one == EUP || one == EDOWN;
      case ERIGHT: return one == EUP || one == EDOWN;
      default: return false;
    }
  }

  //Assume one and another are adjacent
  static void getRelatedBoxCorner(EBoundaryType one, EBoundaryType another,
          Float xMIN,Float xMAX,Float yMIN,Float yMAX,Point2 &corner){
    if(!isAdjacent(one,another)){
      SLog(EError, "Nonadjacent boundaries are applied to getRelatedBoxCorner function");
    }

    if((one == EUP && another == ELEFT) || (another == EUP && one == ELEFT)){
      corner = Point2(xMIN, yMIN);
    }
    else if((one == EUP && another == ERIGHT) || (another == EUP && one == ERIGHT)){
      corner = Point2(xMAX, yMIN);
    }
    else if((one == EDOWN && another == ELEFT) || (another == EDOWN && one == ELEFT)){
      corner = Point2(xMIN, yMAX);
    }
    else if((one == EDOWN && another == ERIGHT) || (another == EDOWN && one == ERIGHT)){
      corner = Point2(xMAX, yMAX);
    }
    return;
  }
};

struct LineEnds{
  public:
    Boundary::EBoundaryType startType;
    Boundary::EBoundaryType endType;
    Point2 start;
    Point2 end;
    Point2 corner;
    bool isInvalid = true;
    bool isAdjacent = false;

  static bool hasSameType(LineEnds one, LineEnds two){
    return ((one.startType == two.startType && one.endType == two.endType) ||
            (one.startType == two.endType && one.endType == two.startType));
  }

};
MTS_NAMESPACE_END

#endif //MITSUBA_IMAGEPLANE_BOUNDARY_H
