#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <learnopengl/shader_m.h>
#include <learnopengl/camera.h>
#include <learnopengl/model.h>

#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

// USER INTERFACE GLOBALS
int LeftButtonDown = 0;    // MOUSE STUFF
int RightButtonDown = 0;


// settings
const unsigned int SCR_WIDTH = 1200;
const unsigned int SCR_HEIGHT = 1200;

// camera
Camera camera(glm::vec3(0.0f, 1.0f, 60.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;
glm::vec3 lightColor = glm::vec3(0.48f, 0.68f, 1.0f);
Shader* lightingShader;

Model* birdObject1;
Model* birdObject2;
const char* birdObjectPath1 = "./bird5.obj";
const char* birdObjectPath2 = "./bird6.obj";

// HOUSE KEEPING
void initGL(GLFWwindow** window);
void setupShader();
void destroyShader();
void createGLPrimitives();
void destroyGLPrimitives();

// CALLBACKS
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void processInput(GLFWwindow* window, int key, int scancode, int action, int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

void DrawFingerBase(glm::mat4 model);
void DrawFingerTip(glm::mat4 model);
void DrawObject(glm::mat4 model);
glm::vec3 birdPosition();
glm::vec3 separation(glm::vec3 boids);
glm::vec3 alignment(glm::vec3 boids);
glm::vec3 cohesion(glm::vec3 boids);
void boidType(int boidName, glm::mat4 model);
glm::mat4 newBird = glm::mat4(1.0f);
void RenderImGui();
bool hasTextures = false;
bool useCursor = true;
bool cohesionFlag = true;
bool separationFlag = true;
bool alignmentFlag = true;
bool obstacleFlag = false;
bool collision = true;
int boundingSensitivity = 10;
int n = 0;
float t = 0.0f;
float r = 0.5f;
float tMax = 100.0f;
float rotateAngle = 0.0f;
float lastd, d, f, timeHit, radius, timeStepRemaining, timeStep;
float h = 0.01;
float stepX = 0;
float stepY = 0; 
float stepZ = 0;
int stepCountX = 0;
int stepCountY = 0;
int stepCountZ = 0;
int tempSensitivity = 10;
glm::vec3 windForce =  glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 windVelocity, lastPos;
glm::vec3 boid = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 boid2 = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 Pos = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 lastVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 currentVelocity = glm::vec3(-0.000001f, 0.000001f, 0.000001f);
glm::vec3 gravity = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 acceleration = gravity;
glm::vec3 returnVec = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 birdBoid = glm::vec3(0.0f, 0.0f, 0.0f);
glm::mat4 boidModel[2000];
glm::vec3 nextPos = Pos + (currentVelocity * (deltaTime * boundingSensitivity));
float groupSize = 16.0f;
float groupSpeed = 0.1f;

float randomSpeed;
float mySpeed;
float nDistance;
int boidAmount = 1000;
float multiply;

int boidsX[2000];
int boidsY[2000];
int boidsZ[2000];
glm::vec3 boids[2000];
glm::vec3 boids2[2000];
float maxspeed = 3;
float maxforce = 0.05f;

void myDisplay()
{
	glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	timeStepRemaining = h;
	timeStep = timeStepRemaining;
	glm::mat4 model = glm::mat4(1.0f);
	model = glm::scale(model, glm::vec3(150.0f, 40.0f, 150.0f));
	if (t < tMax)
	{
		if (timeStepRemaining > 0)
		{
			windVelocity = windForce / deltaTime;
			acceleration = gravity - (windVelocity - (currentVelocity - lastVelocity)) * deltaTime;
			//currentVelocity = currentVelocity + (acceleration * timeStep);
			lastVelocity = currentVelocity;
			lastPos = Pos;
			Pos = Pos - ((lastVelocity + currentVelocity) * timeStep / 2.0f);
			timeStepRemaining = timeStepRemaining - timeStep;
			//currentVelocity = glm::clamp(currentVelocity, glm::vec3(-10.0f, -10.0f, -10.0f), glm::vec3(10.0f, 10.0f, 10.0f));
		}
		n = n + 1;
		t = n * h;
	}
	
	for (int i = 1; i <= boidAmount; ++i) {
		boidModel[i] = glm::translate(boidModel[i], Pos);
		if (separationFlag == true)
		{
			boidModel[i] = glm::translate(boidModel[i], separation(boids[i]) * 0.5f);
		}
		if (cohesionFlag == true)
		{
			boidModel[i] = glm::translate(boidModel[i], cohesion(boids[i]) * 0.1f);
		}
		if (alignmentFlag == true)
		{
			boidModel[i] = glm::translate(boidModel[i], alignment(boids[i]) * 4.0f);
		}
		boidModel[i] = glm::translate(boidModel[i], birdPosition()*0.01f);
	
		DrawObject(boidModel[i]);
		if (obstacleFlag == true)
		{
			DrawFingerTip(model);
		}
	}
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
	RenderImGui();

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

int main()
{
	GLFWwindow* window = NULL;

	initGL(&window);
	setupShader();
	createGLPrimitives();
	srand(time(nullptr));
	for (int i = 0; i < boidAmount; i++)
	{
		boidsX[i] = rand() % 100;
		boidsY[i] = rand() % 100;
		boidsZ[i] = rand() % 100;
		boids[i] = glm::vec3(boidsX[i], boidsY[i], boidsZ[i]);
		boidModel[i] = glm::mat4(1.0f);
		boidModel[i] = glm::scale(boidModel[i], glm::vec3(3.0f, 3.0f, 3.0f));
		boidModel[i] = glm::translate(boidModel[i], boids[i]);
		boidModel[i] = glm::rotate(boidModel[i], glm::radians(rand() % 180 *1.0f), boids[i]);

	}
	while (!glfwWindowShouldClose(window))
	{
		// per-frame time logic
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;
		lightingShader->use();
		lightingShader->setVec3("light.position", camera.Position);
		lightingShader->setVec3("light.direction", camera.Front);
		lightingShader->setVec3("viewPos", camera.Position);
		lightingShader->setVec3("light.ambient", 0.2f, 0.2f, 0.2f);

		lightingShader->setVec3("light.diffuse", 10.0f, 10.0f, 10.0f);
		lightingShader->setVec3("light.specular", 1.0f, 1.0f, 1.0f);
		lightingShader->setFloat("light.constant", 0.1f);
		lightingShader->setFloat("light.linear", 0.0f);
		lightingShader->setFloat("light.quadratic", 0.0009f);

		// material properties
		lightingShader->setFloat("material.shininess", 16.0f);

		// view/projection transformations
		glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
		glm::mat4 view = camera.GetViewMatrix();
		lightingShader->setMat4("projection", projection);
		lightingShader->setMat4("view", view);

		// render
		myDisplay();

		// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
	glfwSwapBuffers(window);
	glfwPollEvents();
	destroyGLPrimitives();
	destroyShader();
	// glfw: terminate, clearing all previously allocated GLFW resources.
	// ------------------------------------------------------------------
	glfwDestroyWindow(window);
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwTerminate();

	return 0;
}

void initGL(GLFWwindow** window)
{
	// glfw: initialize and configure
	// ------------------------------
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X
#endif

	// glfw window creation
	// --------------------
	* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "boid", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		exit(-1);
	}

	glfwMakeContextCurrent(*window);
	glfwSetFramebufferSizeCallback(*window, framebuffer_size_callback);
	glfwSetCursorPosCallback(*window, mouse_callback);
	glfwSetMouseButtonCallback(*window, mouse_button_callback);
	glfwSetScrollCallback(*window, scroll_callback);
	glfwSetKeyCallback(*window, processInput);

	// glad: load all OpenGL function pointers
	// ---------------------------------------
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		exit(-1);
	}

	glEnable(GL_DEPTH_TEST);

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	ImGui::StyleColorsDark();
	ImGui_ImplGlfw_InitForOpenGL(*window, true);
	ImGui_ImplOpenGL3_Init((char *)glGetString(GL_NUM_SHADING_LANGUAGE_VERSIONS));
}

void setupShader()
{
	// Light attributes
	lightingShader = new Shader("light_casters.vs", "light_casters.fs");
	lightingShader->use();
	lightingShader->setVec3("lightColor", lightColor);
}

void destroyShader()
{
	delete lightingShader;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window, int key, int scancode, int action, int mods)
{

	float cameraSpeed = 2.5f * deltaTime;
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		camera.ProcessKeyboard(FORWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		camera.ProcessKeyboard(BACKWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		camera.ProcessKeyboard(LEFT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		camera.ProcessKeyboard(RIGHT, deltaTime);

	if ((glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)) {
		useCursor = !useCursor;
		if (useCursor == true) {
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		}
		else {
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		}
	}
}


// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (firstMouse)
	{
		lastX = (float)xpos;
		lastY = (float)ypos;
		firstMouse = false;
	}

	float xoffset = (float)(xpos - lastX) / SCR_WIDTH * 10.0f;
	float yoffset = (float)(lastY - ypos) / SCR_HEIGHT * 10.0f; // reversed since y-coordinates go from bottom to top

	lastX = (float)xpos;
	lastY = (float)ypos;
	if (RightButtonDown)
	{
		camera.ProcessMouseMovement(xoffset * 200, yoffset * 200);
	}
	if (LeftButtonDown)
	{
		//move bird
	}
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
	{
		LeftButtonDown = 1;
	}
	else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
	{
		LeftButtonDown = 0;
	}
	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
	{
		RightButtonDown = 1;
	}
	else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE)
	{
		RightButtonDown = 0;
	}
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	camera.ProcessMouseScroll(yoffset);
}

class Primitive {
public:
	Primitive() {
		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &vbo);
		glGenBuffers(1, &ebo);
	}
	~Primitive() {
		if (!ebo) glDeleteBuffers(1, &ebo);
		if (!vbo) glDeleteBuffers(1, &vbo);
		if (!VAO) glDeleteVertexArrays(1, &VAO);
	}
	void Draw() {
		glBindVertexArray(VAO);
		glDrawElements(GL_TRIANGLE_STRIP, IndexCount, GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);
	}

protected:
	unsigned int VAO = 0, vbo = 0, ebo = 0;
	unsigned int IndexCount = 0;
	float height = 1.0f;
	float radius[2] = { 1.0f, 1.0f };
};

class Cylinder : public Primitive {
public:
	Cylinder(float bottomRadius = 0.5f, float topRadius = 0.5f, int NumSegs = 16);
};

class Sphere : public Primitive {
public:
	Sphere(int NumSegs = 16);
};

class Plane : public Primitive {
public:
	Plane();
	void Draw() {
		glBindVertexArray(VAO);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, floorTexture);
		glDrawElements(GL_TRIANGLE_STRIP, IndexCount, GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);
	}
private:
	unsigned int floorTexture;
};

Sphere* unitSphere;
Plane* groundPlane;
Cylinder* unitCylinder;
Cylinder* unitCone;

void createGLPrimitives()
{
	unitSphere = new Sphere();
	groundPlane = new Plane();
	unitCylinder = new Cylinder();
	unitCone = new Cylinder(0.5, 0);

	// Load Object Model
	birdObject1 = new Model(birdObjectPath1);
	birdObject2 = new Model(birdObjectPath2);
	hasTextures = (birdObject1->textures_loaded.size() == 0) ? 0 : 1;

}
void destroyGLPrimitives()
{
	delete unitSphere;
	delete groundPlane;
	delete unitCylinder;
	delete unitCone;

	delete birdObject1;
	delete birdObject2;
}


void DrawFingerBase(glm::mat4 model)
{
	glm::mat4 Base = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.2f, 0.0f));
	Base = glm::scale(Base, glm::vec3(0.05f, 0.3f, 0.05f));
	glm::mat4 InBase = glm::inverse(Base);

	lightingShader->use();
	Base = model * Base;
	lightingShader->setMat4("model", Base);
	lightingShader->setVec3("ObjColor", glm::vec3(1.0f, 0.0f, 0.0f));
	lightingShader->setInt("hasTextures", false);
	unitCylinder->Draw();

	glm::mat4 Mat1 = glm::translate(InBase, glm::vec3(0.0f, 0.35f, 0.0f));
	Mat1 = glm::scale(Mat1, glm::vec3(0.05f, 0.05f, 0.05f));

	Mat1 = Base * Mat1;
	lightingShader->setMat4("model", Mat1);
	lightingShader->setVec3("ObjColor", glm::vec3(1.0f, 1.0f, 0.0f));
	unitSphere->Draw();
}
void DrawFingerTip(glm::mat4 model)
{
	glm::mat4 Base = glm::scale(glm::mat4(1.0f), glm::vec3(0.05f, 0.25f, 0.05f));
	Base = glm::translate(Base, glm::vec3(0.0f, 0.4f, 0.0f));

	lightingShader->use();
	Base = model * Base;
	lightingShader->setMat4("model", Base);
	lightingShader->setVec3("ObjColor", glm::vec3((1.0f, 0.0f, 1.0f)));
	lightingShader->setInt("hasTextures", false);
	unitCone->Draw();
}

void DrawObject(glm::mat4 model)
{
	lightingShader->use();
	lightingShader->setMat4("model", model);
	lightingShader->setVec3("ObjColor", glm::vec3(1.0f, 1.0f, 1.0f));
	lightingShader->setInt("hasTextures", hasTextures);
	birdObject1->Draw(*lightingShader);
}

void boidType(int boidName, glm::mat4 model) {
	boidName = glm::clamp(boidName, 0, 3);

	switch (boidName) {
	case 0:
		lightingShader->use();
		lightingShader->setMat4("model", model);
		lightingShader->setVec3("ObjColor", glm::vec3(1.0f, 1.0f, 1.0f));
		lightingShader->setInt("hasTextures", hasTextures);
		birdObject1->Draw(*lightingShader);
		break;
	case 1:
		lightingShader->use();
		lightingShader->setMat4("model", model);
		lightingShader->setVec3("ObjColor", glm::vec3(1.0f, 1.0f, 0.0f));
		lightingShader->setInt("hasTextures", hasTextures);
		birdObject2->Draw(*lightingShader);
		break;
	}
}
glm::vec3 birdPosition() {
	if (stepCountX <= 0) {
		stepX = 0.0f;
		if (((nextPos[0] >= 10) && (currentVelocity[0] > 0)) || ((nextPos[0] <= 0) && (currentVelocity[0] < 0))) {
			stepX = currentVelocity[0];
			stepCountX = boundingSensitivity;
		}
	}
	if (stepCountX > 0) {
		returnVec += glm::vec3(-(2 * stepX) / boundingSensitivity, 0.0f, 0.0f);
		stepCountX--;
	}

	if (stepCountY <= 0) {
		stepY = 0;
		if (((nextPos[1] >= 10) && (currentVelocity[1] >= 0)) || ((nextPos[1] <= 0) && (currentVelocity[1] <= 0))) {
			stepY = currentVelocity.y;
			stepCountY = boundingSensitivity;
		}
	}
	if (stepCountY > 0) {
		returnVec += glm::vec3(0.0f, -(2 * stepY) / boundingSensitivity, 0.0f);
		stepCountY--;
	}

	if (stepCountZ <= 0) {
		stepZ = 0;
		if (((nextPos[2] >= 10) && (currentVelocity.z >= 0)) || ((nextPos[2] <= 0) && (currentVelocity.z <= 0))) {
			;
			stepZ = currentVelocity.z;
			stepCountZ = boundingSensitivity;
		}
	}
	if (stepCountZ > 0) {
		returnVec += glm::vec3(0.0f, 0.0f, -(2 * stepZ) / boundingSensitivity);
		stepCountZ--;
	}

	if ((stepCountX > 0) || (stepCountY > 0) || (stepCountZ > 0)) {
		collision = true;
	}
	else {
		collision = false;
		boundingSensitivity = tempSensitivity;
	}

	if (Pos[0] > 10) {
		currentVelocity[0] = -10;
		stepCountX = 0;
	}
	else if (Pos[0] < -1) {
		currentVelocity[0] = 10;
		stepCountX = 0;
	}

	if (Pos[1] > 10) {
		currentVelocity[1] = -10;
		stepCountY = 0;
	}
	else if (Pos[1] < -1) {
		currentVelocity[1] = 10;
		stepCountY = 0;
	}

	if (Pos[2] > 10) {
		currentVelocity[2] = -10;
		stepCountZ = 0;
	}
	else if (Pos[2] < -1) {
		currentVelocity[2] = 10;
		stepCountZ = 0;
	}
	return returnVec;
}
glm::vec3 separation(glm::vec3 boids) {
	glm::vec3 separationVelocity = glm::vec3(0.0f, -1.0f, 0.0f);
	int count = 0;
	int neighbour = 0;
	float desiredseparation = 100.0;
	glm::vec3 steer = glm::vec3(0.0f, 0.0f, -5.0f);
	glm::vec3 localPosition = Pos;
	for (int i = 0; i < 1000; i++)
	{
		glm::vec3 targetPosition = boids;
		if (i == 0)
		{
			targetPosition = lastVelocity;
		}
		else
		{
			nDistance = glm::length(targetPosition - localPosition);
			if (nDistance < desiredseparation)
			{
				separationVelocity += 0.001f*(localPosition - targetPosition);
				neighbour++;
			}
		}
	}
	if (neighbour != 0) {
		separationVelocity /= neighbour;
	}
	if (glm::length(separationVelocity) > 0.0f) {
		separationVelocity = glm::normalize(separationVelocity);
		separationVelocity = separationVelocity * maxspeed;
		separationVelocity -= boids;
		if (separationVelocity[0] > maxforce || separationVelocity[1] > maxforce || separationVelocity[2] > maxforce)
		{
			separationVelocity = glm::vec3(maxforce, maxforce, maxforce);
		}
	}
	else if (separationVelocity[0] < 0.3f)
	{
		separationVelocity += boids;
	}
	return separationVelocity * 0.02f;
}
glm::vec3 alignment(glm::vec3 boids) {
	glm::vec3 alignmentVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
	int neighbour = 0;
	glm::vec3 targetPosition = boids;
	glm::vec3 localPosition = Pos;
	for (int i = 0; i < 1000; i++)
	{
		if (i == 0)
		{
			targetPosition = boids;
		}
		else
		{
			nDistance = glm::length(targetPosition - localPosition);
			if (nDistance < groupSize)
			{
				alignmentVelocity += lastVelocity;
				neighbour++;
			}
		}
	}
	if (neighbour != 0) {
		alignmentVelocity /= neighbour;
	}
	if (glm::length(alignmentVelocity) > 0.0f) {
		alignmentVelocity = glm::normalize(alignmentVelocity);
	}

	return alignmentVelocity*4.0f;
}
glm::vec3 cohesion(glm::vec3 boids) {
	glm::vec3 cohesionVelocity = glm::vec3(0.0f, -0.1f, -7.0f);
	int neighbour = 0;
	int neighbourdist = 10;
	glm::vec3 targetPosition = boids;
	glm::vec3 localPosition = Pos;
	for (int i = 0; i < glm::length(boids); i++)
	{
		float d = glm::length(boids);
		if (d > 0 && d < neighbourdist)
		{
			cohesionVelocity += boids*0.1f;
			neighbour++;
		}
	}
	if (neighbour > 0) {
		cohesionVelocity /= neighbour;
	}
	return cohesionVelocity*0.05f;
}

void RenderImGui() {
	ImGui::Begin("boid");
	ImGui::Checkbox("Cohesion", &cohesionFlag);
	ImGui::Checkbox("separation", &separationFlag);
	ImGui::Checkbox("Alignment", &alignmentFlag);
	ImGui::Text("bird count :");
	ImGui::SameLine();
	ImGui::Text(std::to_string(boidAmount).c_str());
	if (ImGui::Button("add bird")) {
		boidAmount += 200;
	}
	ImGui::SameLine();
	if (ImGui::Button("reduce bird")) {
		boidAmount -= 200;
	}
	if (ImGui::Button("set obstacle")) {
		if (obstacleFlag == true)
		{
			obstacleFlag = false;
		}
		else
		{
			obstacleFlag = true;
		}
	}
	if (ImGui::Button("reset")) {
		currentVelocity = glm::vec3(-0.0001f, -0.0001f, 0.0001f);
		Pos = glm::vec3(0.0f, 0.0f, 0.0f);
		lastVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
		for (int i = 0; i < 1000; i++)
		{
			boidModel[i] = glm::mat4(1.0f);
			boidsX[i] = rand() % 100;
			boidsY[i] = rand() % 100;
			boidsZ[i] = rand() % 100;
			boids[i] = glm::vec3(boidsX[i], boidsY[i], boidsZ[i]);
			boidModel[i] = glm::translate(boidModel[i], boids[i]);
			boidModel[i] = glm::rotate(boidModel[i], glm::radians(rand() % 180 * 1.0f), boids[i]);

		}
	}

	ImGui::End();
}

Sphere::Sphere(int NumSegs)
{
	std::vector<glm::vec3> positions;
	std::vector<glm::vec3> normals;
	std::vector<unsigned int> indices;

	const unsigned int X_SEGMENTS = NumSegs;
	const unsigned int Y_SEGMENTS = NumSegs;
	const float PI = (float)3.14159265359;

	for (unsigned int y = 0; y <= Y_SEGMENTS; ++y)
	{
		for (unsigned int x = 0; x <= X_SEGMENTS; ++x)
		{
			float xSegment = (float)x / (float)X_SEGMENTS;
			float ySegment = (float)y / (float)Y_SEGMENTS;
			float xPos = std::cos(xSegment * 2.0f * PI) * std::sin(ySegment * PI);
			float yPos = std::cos(ySegment * PI);
			float zPos = std::sin(xSegment * 2.0f * PI) * std::sin(ySegment * PI);

			positions.push_back(glm::vec3(xPos, yPos, zPos));
			normals.push_back(glm::vec3(xPos, yPos, zPos));
		}
	}

	bool oddRow = false;
	for (unsigned int y = 0; y < Y_SEGMENTS; ++y)
	{
		if (!oddRow)
		{
			for (unsigned int x = 0; x <= X_SEGMENTS; ++x)
			{
				indices.push_back(y * (X_SEGMENTS + 1) + x);
				indices.push_back((y + 1) * (X_SEGMENTS + 1) + x);
			}
		}
		else
		{
			for (int x = X_SEGMENTS; x >= 0; --x)
			{
				indices.push_back((y + 1) * (X_SEGMENTS + 1) + x);
				indices.push_back(y * (X_SEGMENTS + 1) + x);
			}
		}
		oddRow = !oddRow;
	}

	IndexCount = (unsigned int)indices.size();

	std::vector<float> data;
	for (int i = 0; i < positions.size(); ++i)
	{
		data.push_back(positions[i].x);
		data.push_back(positions[i].y);
		data.push_back(positions[i].z);
		if (normals.size() > 0)
		{
			data.push_back(normals[i].x);
			data.push_back(normals[i].y);
			data.push_back(normals[i].z);
		}
	}

	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), &data[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);
	GLsizei stride = (3 + 3) * sizeof(float);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));

}


// utility function for loading a 2D texture from file
// ---------------------------------------------------
unsigned int loadTexture(char const* path)
{
	unsigned int textureID;
	glGenTextures(1, &textureID);

	int width, height, nrComponents;
	unsigned char* data = stbi_load(path, &width, &height, &nrComponents, 0);
	if (data)
	{
		GLenum format;
		if (nrComponents == 1)
			format = GL_RED;
		else if (nrComponents == 3)
			format = GL_RGB;
		else if (nrComponents == 4)
			format = GL_RGBA;

		glBindTexture(GL_TEXTURE_2D, textureID);
		glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
		glGenerateMipmap(GL_TEXTURE_2D);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, format == GL_RGBA ? GL_CLAMP_TO_EDGE : GL_REPEAT); // for this tutorial: use GL_CLAMP_TO_EDGE to prevent semi-transparent borders. Due to interpolation it takes texels from next repeat 
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, format == GL_RGBA ? GL_CLAMP_TO_EDGE : GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		stbi_image_free(data);
	}
	else
	{
		std::cout << "Texture failed to load at path: " << path << std::endl;
		stbi_image_free(data);
	}

	return textureID;
}


Plane::Plane()
{
	float data[] = {
		// positions           // normals          // texcoords
		-10.0f, 0.0f, -10.0f,  0.0f, 1.0f,  0.0f,  0.0f,  0.0f,
		 10.0f, 0.0f, -10.0f,  0.0f, 1.0f,  0.0f,  10.0f, 0.0f,
		 10.0f, 0.0f,  10.0f,  0.0f, 1.0f,  0.0f,  10.0f, 10.0f,
		-10.0f, 0.0f,  10.0f,  0.0f, 1.0f,  0.0f,  0.0f,  10.0f
	};
	unsigned int indices[] = { 0, 1, 3, 2 };

	IndexCount = sizeof(indices) / sizeof(unsigned int);

	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(data), data, GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	GLsizei stride = (3 + 3 + 2) * sizeof(float);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));

	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, (void*)(6 * sizeof(float)));

	glBindVertexArray(0);

	floorTexture = loadTexture("./wood.png");
	lightingShader->use();
	lightingShader->setInt("texture1", floorTexture);

}

Cylinder::Cylinder(float bottomRadius, float topRadius, int NumSegs)
{
	radius[0] = bottomRadius; radius[1] = topRadius;

	std::vector<glm::vec3> base;
	std::vector<glm::vec3> positions;
	std::vector<glm::vec3> normals;
	std::vector<unsigned int> indices;

	//a circle
	const float PI = (float)3.14159265359;
	float sectorStep = 2 * PI / NumSegs;				// Angle increasing
	float sectorAngle;									// radian

	for (int i = 0; i <= NumSegs; ++i)
	{
		sectorAngle = i * sectorStep;
		float xPos = std::sin(sectorAngle);
		float yPos = 0;
		float zPos = std::cos(sectorAngle);

		base.push_back(glm::vec3(xPos, yPos, zPos));
	}

	//put side of cylinder
	for (int i = 0; i < 2; ++i)
	{
		float h = -height / 2.0f + i * height;			// height from -h/2 to h/2   

		for (int j = 0; j <= NumSegs; ++j)
		{
			positions.push_back(glm::vec3(base[j].x * radius[i], h, base[j].z * radius[i]));
			normals.push_back(glm::vec3(base[j].x, h, base[j].z));
		}
	}

	//the starting index for the base/top surface
	//NOTE: it is used for generating indices later
	int baseCenterIndex = (int)positions.size();
	int topCenterIndex = baseCenterIndex + NumSegs + 1; // include center vertex

	//put base and top circles
	for (int i = 0; i < 2; ++i)
	{
		float h = -height / 2.0f + i * height;
		float ny = (float)-1 + i * 2;

		// center point
		positions.push_back(glm::vec3(0, h, 0));		// height from -h/2 to h/2
		normals.push_back(glm::vec3(0, ny, 0));			// z value of normal; -1 to 1

		for (int j = 0; j < NumSegs; ++j)
		{
			positions.push_back(glm::vec3(base[j].x * radius[i], h, base[j].z * radius[i]));
			normals.push_back(glm::vec3(0, ny, 0));
		}
	}

	//Indexing
	int k1 = 0;											// 1st vertex index at base
	int k2 = NumSegs + 1;								// 1st vertex index at top

	// indices for the side surface
	for (int i = 0; i < NumSegs; ++i, ++k1, ++k2)
	{
		// 2 triangles per sector
		// k1 => k1+1 => k2
		indices.push_back(k1);
		indices.push_back(k1 + 1);
		indices.push_back(k2);

		// k2 => k1+1 => k2+1
		indices.push_back(k2);
		indices.push_back(k1 + 1);
		indices.push_back(k2 + 1);
	}

	//indices for the base surface
	//NOTE: baseCenterIndex and topCenterIndices are pre-computed during vertex generation
	//      please see the previous code snippet
	for (int i = 0, k = baseCenterIndex + 1; i < NumSegs; ++i, ++k)
	{
		if (i < NumSegs - 1)
		{
			indices.push_back(baseCenterIndex);
			indices.push_back(k + 1);
			indices.push_back(k);
		}
		else // last triangle
		{
			indices.push_back(baseCenterIndex);
			indices.push_back(baseCenterIndex + 1);
			indices.push_back(k);
		}
	}

	// indices for the top surface
	for (int i = 0, k = topCenterIndex + 1; i < NumSegs; ++i, ++k)
	{
		if (i < NumSegs - 1)
		{
			indices.push_back(topCenterIndex);
			indices.push_back(k);
			indices.push_back(k + 1);
		}
		else // last triangle
		{
			indices.push_back(topCenterIndex);
			indices.push_back(k);
			indices.push_back(topCenterIndex + 1);
		}
	}
	IndexCount = (unsigned int)indices.size();

	std::vector<float> data;
	for (int i = 0; i < positions.size(); ++i)
	{
		data.push_back(positions[i].x);
		data.push_back(positions[i].y);
		data.push_back(positions[i].z);

		if (normals.size() > 0)
		{
			data.push_back(normals[i].x);
			data.push_back(normals[i].y);
			data.push_back(normals[i].z);
		}
	}
	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), &data[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);
	GLsizei stride = (3 + 3) * sizeof(float);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));
}

