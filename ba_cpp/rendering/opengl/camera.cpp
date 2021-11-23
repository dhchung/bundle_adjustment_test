#include "camera.h"
#include <iostream>

Camera::Camera(){
    Defaultpitch = 90.0;
}

Camera::Camera(glm::vec3 position,
               glm::vec3 up,
               float yaw, 
               float pitch):
               Front(glm::vec3(1.0f, 0.0f, 0.0f)), 
               OriginalSpeed(SPEED), 
               MouseSensitivity(SENSITIVITY), 
               Zoom(ZOOM) {
    Position = position;
    WorldUp = up;
    Yaw = yaw;
    Pitch = pitch;
    MovementSpeed = OriginalSpeed;
    FastSpeed = OriginalSpeed*5;
    Defaultpitch = 90.0f;
    updateCameraVectors();
}

Camera::Camera(float posX,
               float posY,
               float posZ,
               float upX,
               float upY,
               float upZ,
               float yaw,
               float pitch):
               Front(glm::vec3(1.0f, 0.0f, 0.0f)), 
               MovementSpeed(SPEED), 
               MouseSensitivity(SENSITIVITY), 
               Zoom(ZOOM){
    Position = glm::vec3(posX, posY, posZ);
    WorldUp = glm::vec3(upX, upY, upZ);
    Yaw = yaw;
    Pitch = pitch;
    Defaultpitch = 90.0f;
    updateCameraVectors();
}

glm::mat4 Camera::GetViewMatrix() {
    return glm::lookAt(Position, Position + Front, Up);
}

glm::mat4 Camera::GetViewMatrix_right() {
    glm::vec3 Position_R;
    glm::vec4 rel_pos_R(-1.0, 0.0, 0.0, 1.0);
    glm::mat4 trans(1.0);
    trans = glm::rotate(trans, glm::radians(-Yaw), glm::vec3(0.0, 1.0, 0.0));
    trans = glm::rotate(trans, glm::radians(-Pitch), glm::vec3(1.0, 0.0, 0.0));
    // trans = glm::translate(trans, glm::vec3(1.0f, 0.0f, 0.0f));
    glm::vec4 temp(1.0);
    temp = trans*rel_pos_R;
    Position_R.x = temp.x;
    Position_R.y = temp.y;
    Position_R.z = temp.z;

    Position_R += Position;
    return glm::lookAt(Position_R, Position_R + Front, Up);
}

void Camera::ProcessKeyboard(Camera_Movement direction, float deltaTime) {
    float velocity = MovementSpeed * deltaTime;
    if(direction == FORWARD) {
        Position += Front * velocity;
        // Position += glm::vec3(Front.x, 0.0f, Front.z)*velocity;
    }
    if(direction == BACKWARD) {
        Position -= Front * velocity;
        // Position -= glm::vec3(Front.x, 0.0f, Front.z)*velocity;
    }
    if(direction == LEFT) {
        Position -= Right * velocity;
    }
    if(direction == RIGHT) {
        Position += Right * velocity;
    }
    
}

void Camera::ProcessInput(float dx, float dy, float dz,
                          float droll = 0.0f, float dpitch = 0.0f, float dyaw = 0.0f) {

    // glm::vec4 temp(Position.x, Position.y, Position.z, 1.0f);
    // glm::mat4 trans(1.0f);
    // trans = glm::rotate(trans, glm::radians(droll), glm::vec3(0.0f, 0.0f, 1.0f));
    // trans = glm::rotate(trans, glm::radians(dpitch), glm::vec3(1.0f, 0.0f, 0.0f));
    // trans = glm::rotate(trans, glm::radians(dyaw), glm::vec3(0.0f, 1.0f, 0.0f));
    // trans = glm::translate(trans, glm::vec3(dx, dy, dz));
    // // trans = glm::inverse(trans);
    // temp = trans*temp;
    // Position.x = temp.x;
    // Position.y = temp.y;
    // Position.z = temp.z;

}


void Camera::ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch) {
    xoffset *= MouseSensitivity;
    yoffset *= MouseSensitivity;

    Yaw += xoffset;
    Pitch -= yoffset;

    if(constrainPitch) {
        if(fabs(Pitch) > 89.0f) {
            Pitch = 89.0f * Pitch/fabs(Pitch);
        }
    }

    // std::cout<<"Pitch: "<<Pitch<<", Yaw: "<<Yaw<<std::endl;
    updateCameraVectors();
}

void Camera::ProcessMouseScroll(float yoffset) {
    if (Zoom >= 1.0f && Zoom <= 45.0f) {
        Zoom -= yoffset;
    }
    if (Zoom <= 1.0f) {
        Zoom = 1.0f;
    }
    if (Zoom >= 45.0f) {
        Zoom = 45.0f;
    }
}

void Camera::updateCameraVectors() {
    glm::vec4 rel_pos_R(1.0, 0.0, 0.0, 1.0);
    glm::mat4 trans(1.0);
    // trans = glm::translate(trans, glm::vec3(1.0f, 0.0f, 0.0f));
    trans = glm::rotate(trans, glm::radians(-Pitch), glm::vec3(1.0, 0.0, 0.0));
    trans = glm::rotate(trans, glm::radians(Yaw), glm::vec3(0.0, 1.0, 0.0));
    glm::vec4 temp(0.0, 0.0, 1.0, 1.0);
    temp = trans*rel_pos_R;

    glm::vec3 front;
    front.x = temp.x;
    front.y = temp.y;
    front.z = temp.z;
    
    Front = glm::normalize(front);

    Right = glm::normalize(glm::cross(Front, WorldUp));
    Up = glm::normalize(glm::cross(Right, Front));
}
