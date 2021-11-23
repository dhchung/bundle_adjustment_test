#include "opengl_rendering.h"


OpenglRendering::OpenglRendering(std::string window_name){
    w_name = window_name;
    screenWidth = 1600;
    screenHeight = 900;

    camera = new Camera(glm::vec3(-2.0f, 0.0f, -1.0f), glm::vec3(0.0f,-1.0f,0.0f));

    lastX = (float)screenWidth/2.0f;
    lastY = (float)screenHeight/2.0f;
    firstMouse = true;

    deltaTime = 0.0f;
    lastFrame = 0.0f;
    

    imgs.clear();
}

OpenglRendering::~OpenglRendering(){

}

void OpenglRendering::init_opengl(){
    glfwInit(); //Initialize GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3); //OpenGL 3.3
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); //Use only core profiles of opengl

    
    window = glfwCreateWindow(screenWidth,screenHeight, w_name.c_str(), NULL, NULL);
    if(window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetWindowUserPointer(window, this);


    

    glfwMakeContextCurrent(window); //Tell GLFW to setup "window context" as primary context for current thread

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { //glfwGetProcAddress : Get correct functions regarding to OS or compile environment
        std::cout<< "Failed to initialize GLAD" << std::endl;
    }

    point_shader.InitShader("../rendering/shaders/point/vertex_shader.vs", "../rendering/shaders/point/fragment_shader.fs");
    plane_shader.InitShader("../rendering/shaders/plane/with_texture/vertex_shader.vs", "../rendering/shaders/plane/with_texture/fragment_shader.fs");

    surfel_shader.InitShader("../rendering/shaders/surfel/vertex_shader.vs", "../rendering/shaders/surfel/fragment_shader.fs");

    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glGenVertexArrays(1, &VAO);
    glEnable(GL_DEPTH_TEST);  

}

void OpenglRendering::framebuffer_size_callback(GLFWwindow* window, int width, int height) { // Change viewport if the user change the size of the window
    glViewport(0, 0, width, height);
}

void OpenglRendering::processInput() {
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }
}

glm::mat4 OpenglRendering::eigen_mat4_to_glm_mat4(Eigen::Matrix4f & e_mat4){

    glm::mat4 result;
    for(int i=0;i<4;++i){
        for(int j=0;j<4;++j){
            result[j][i] = e_mat4(i,j);
        }
    }
    return result;
}

void OpenglRendering::terminate(){
    glfwTerminate();
}

void OpenglRendering::clear_window(){
    processInput();
    glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  
}

void OpenglRendering::draw_points(std::vector<Point3D> & pointcloud) {
    clear_window();
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    next_img = false;

    int data_size = pointcloud.size();
    float vertices[6*data_size];

    for(int i = 0; i<data_size; ++i){
        vertices[i*6+0] = pointcloud[i].x;
        vertices[i*6+1] = pointcloud[i].y;
        vertices[i*6+2] = pointcloud[i].z;

        vertices[i*6+3] = float(pointcloud[i].r)/255.0f;
        vertices[i*6+4] = float(pointcloud[i].g)/255.0f;
        vertices[i*6+5] = float(pointcloud[i].b)/255.0f;
    }

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view = glm::mat4(1.0f);
    glm::mat4 projection = glm::mat4(1.0f);

    point_shader.use();
    projection = glm::perspective(glm::radians(camera->Zoom), (float)screenWidth/(float)screenHeight, 0.1f, 100.0f);


    while(!next_img) {
        processInput_end();
        glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  

        view = camera->GetViewMatrix();
        point_shader.setMat4("model", model);
        point_shader.setMat4("view", view);
        point_shader.setMat4("projection", projection);
        point_shader.setBool("use_texture_in", false);

        glBindVertexArray(VAO);
        glPointSize(1.0);
        glDrawArrays(GL_POINTS, 0, data_size);
        glBindVertexArray(0);
        draw_axis(2.0, 10.0, &point_shader);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

}


void OpenglRendering::draw_axis(float line_length, float line_width, Shader * shader){
    float vertices[]{
        0.0f, 0.0f, 0.0f,                   1.0f, 0.0f, 0.0f,//v0
        line_length*1.0f, 0.0f, 0.0f,       1.0f, 0.0f, 0.0f,//vx
        0.0f, 0.0f, 0.0f,                   0.0f, 1.0f, 0.0f,//v0
        0.0f, line_length*1.0f, 0.0f,       0.0f, 1.0f, 0.0f,//vy
        0.0f, 0.0f, 0.0f,                   0.0f, 0.0f, 1.0f,//v0
        0.0f, 0.0f, line_length*1.0f,       0.0f, 0.0f, 1.0f//vz
    };

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    glm::mat4 model = glm::mat4(1.0f);

    shader->setMat4("model", model);

    glLineWidth(line_width);
    // glDrawElements(GL_LINES, 6, GL_UNSIGNED_BYTE, 0);
    glDrawArrays(GL_LINES, 0, 6);
    
}

void OpenglRendering::draw_camera(float f, float cx, float cy, glm::mat4 &model_T, float line_length, float line_width, std::vector<float> &color, Shader * shader){

    float x = line_length;

    float y = cx/f*line_length;
    float z = cy/f*line_length;

    float vertices[]{
        0.0f, 0.0f, 0.0f,                   color[0], color[1], color[2],//v0
        x, y, -z,                           color[0], color[1], color[2],

        0.0f, 0.0f, 0.0f,                   color[0], color[1], color[2],//v0
        x, y, z,                            color[0], color[1], color[2],

        0.0f, 0.0f, 0.0f,                   color[0], color[1], color[2],//v0
        x, -y, -z,                          color[0], color[1], color[2],

        0.0f, 0.0f, 0.0f,                   color[0], color[1], color[2],//v0
        x, -y, z,                           color[0], color[1], color[2],

        x, y, -z,                           color[0], color[1], color[2],//v0
        x, -y, -z,                          color[0], color[1], color[2],

        x, -y, -z,                          color[0], color[1], color[2],
        x, -y, z,                           color[0], color[1], color[2],

        x, -y, z,                           color[0], color[1], color[2],
        x, y, z,                            color[0], color[1], color[2],

        x, y, z,                            color[0], color[1], color[2],
        x, y, -z,                           color[0], color[1], color[2]
    };
    

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    glm::mat4 model = model_T;

    shader->setMat4("model", model);

    glLineWidth(line_width);
    glDrawArrays(GL_LINES, 0, 16);
}


void OpenglRendering::draw_arrow(glm::mat4 &model_T, float line_length, float arrow_length, float line_width, std::vector<float> &color, Shader * shader){

    float arrow_sqrt = arrow_length / sqrt(2.0f);
    float arrow_short = line_length - arrow_sqrt;

    float vertices[]{
        0.0f, 0.0f, 0.0f,                   color[0], color[1], color[2],//v0
        line_length, 0.0f, 0.0f,            color[0], color[1], color[2],

        line_length, 0.0f, 0.0f,            color[0], color[1], color[2],//v0
        arrow_short, arrow_sqrt, 0,         color[0], color[1], color[2],

        line_length, 0.0f, 0.0f,            color[0], color[1], color[2],//v0
        arrow_short, -arrow_sqrt, 0,        color[0], color[1], color[2]
    };
    

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    glm::mat4 model = model_T;

    shader->setMat4("model", model);

    glLineWidth(line_width);
    // glDrawElements(GL_LINES, 6, GL_UNSIGNED_BYTE, 0);
    glDrawArrays(GL_LINES, 0, 6);
}

void OpenglRendering::processInput_end(){
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }

    float currentFrame = glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    if(glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
        camera->MovementSpeed = camera->FastSpeed;
    } else {
        camera->MovementSpeed = camera->OriginalSpeed;
    }

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        camera->ProcessKeyboard(FORWARD, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
        camera->ProcessKeyboard(BACKWARD, deltaTime);
    }    
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        camera->ProcessKeyboard(LEFT, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        camera->ProcessKeyboard(RIGHT, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS) {
        next_img = true;
    }

}

void OpenglRendering::mouse_callback(GLFWwindow * window, double xpos, double ypos) {

    OpenglRendering *ogl_pointer =
         static_cast<OpenglRendering*>(glfwGetWindowUserPointer(window));
    ogl_pointer->mouse_callback_function(xpos, ypos);

}

void OpenglRendering::scroll_callback(GLFWwindow * window, double xoffset, double yoffset) {
    OpenglRendering *ogl_pointer =
         static_cast<OpenglRendering*>(glfwGetWindowUserPointer(window));
    ogl_pointer->scroll_callback_function(xoffset, yoffset);
}

void OpenglRendering::mouse_callback_function(double xpos, double ypos) {
    if(firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos-lastX;
    float yoffset = lastY - ypos;
    lastX = xpos;
    lastY = ypos;

    camera->ProcessMouseMovement(xoffset, yoffset);
}

void OpenglRendering::scroll_callback_function(double xoffset, double yoffset){
    camera->ProcessMouseScroll(yoffset);
}


Eigen::Matrix3f OpenglRendering::skew_symmetric(Eigen::Vector3f& vector){
    Eigen::Matrix3f result;
    result(0, 0) = 0.0;
    result(0, 1) = -vector(2);
    result(0, 2) = vector(1);
    result(1, 0) = vector(2);
    result(1, 1) = 0;
    result(1, 2) = -vector(0);
    result(2, 0) = -vector(1);
    result(2, 1) = vector(0);
    result(2, 2) = 0;
    return result;
}

void OpenglRendering::get_camera_properties(cv::Mat & CameraMatrix){
    focal_length = CameraMatrix.at<double>(0,0);
    cx = CameraMatrix.at<double>(0,2);
    cy = CameraMatrix.at<double>(1,2);
}