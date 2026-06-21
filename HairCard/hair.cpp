#include "hair.h"

hair::hair()
{
}

hair::hair(float length, int count)
{
	hairstrand* h = new hairstrand(length, count);
	myHair.push_back(h);
	cout << myHair.size() << endl;
}

hair::hair(float length, int count, float radius)
{
	float R = radius;
	for (int i = -R; i <= R; i++)
	{
		for (int j = -R; j <= R; j++)
		{
			if ((float)i * (float)i + (float)j * (float)j == R * R)
			{
				Vec3<float> InitPos(i, 0.0f, j);
				hairstrand* h = new hairstrand(InitPos, length, count);
				myHair.push_back(h);
			}
		}
	}
	cout << myHair.size() << endl;
}

hair::hair(float length, int count, int line_length)
{
	Vec3<float> InitPos(0.0f, 0.0f, 0.0f);
	for (int i = 0; i < line_length; i++)
	{
		InitPos.SetX(InitPos.GetX() + 1.0f);
		InitPos.SetZ(InitPos.GetZ() + 1.0f);
		hairstrand* h = new hairstrand(InitPos, length, count);
		myHair.push_back(h);
	}
	cout << myHair.size() << endl;

}

hair::hair(string meshFilename, float length, int count)
{
	myMesh = new modelLoad();

	if (myMesh->LoadMeshfile(meshFilename))
	{
		myMesh->BuildCollisionGrid(1.0f);
		Vec3<float> center = myMesh->Center();

		for (auto& v : myMesh->vertexArray)
		{
			if (v.position.GetY() < 2.0f)
				continue;

			// vary length a bit per root so the hair is not uniform
			float frac = v.position.GetX() - (float)(int)v.position.GetX();
			float len = length * (0.85f + 0.30f * fabsf(frac));

			hairstrand* h = new hairstrand(v.position, len, count);
			if (v.normal.LengthSquared() > 1e-6f)
				h->SetInitDir(v.normal);	// grow along the scalp normal
			h->SetHeadCenter(center);
			h->SetCollider(myMesh);
			h->Init();
			myHair.push_back(h);
		}
		cout << myHair.size() << endl;
	}
}

hair::~hair()
{
}

void hair::Init()
{
	for (auto& hairstrand : myHair)
	{
		hairstrand->Init();
	}
}

void hair::Simulation(float dt)
{
	for (auto& hairstrand : myHair)
	{
		hairstrand->Simulation(dt);
	}
}

void hair::Draw()
{
	if (myMesh != NULL)
	{
		myMesh->RenderMesh();
	}
	for (auto& hairstrand : myHair)
	{
		hairstrand->Draw();
	}

}

void hair::ApplyWindtoHair(Vec3<float> wind)
{
	for (auto& hairstrand : myHair)
	{
		hairstrand->ApplyWind(wind);
	}
}
