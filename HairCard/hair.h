#pragma once
#include "modelLoad.h"
#include "hairstrand.h"

class hair
{
public:
	modelLoad* myMesh = nullptr;
	vector<hairstrand*> myHair;
public:
	hair();
	hair(float length, int count);
	hair(float length, int count, float radius);
	hair(float length, int count, int line_length);
	hair(string meshFilename, float length, int count);
	~hair();
public:
	void Init();
	void Simulation(float dt);
	void Draw();
	void ApplyWindtoHair(Vec3<float> wind);

};
