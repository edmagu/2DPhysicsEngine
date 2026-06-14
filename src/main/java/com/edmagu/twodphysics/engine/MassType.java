package com.edmagu.twodphysics.engine;

public enum MassType {

    // Normal bodies can move, rotate, collide, and react to forces.
    NORMAL,

    // Infinite-mass bodies act like static objects such as floors and walls.
    INFINITE
}