#include "AppClass.h"
void Application::InitVariables(void)
{
	m_pCameraMngr->SetPositionTargetAndUpward(vector3(0.0f, 0.0f, 30.0f), ZERO_V3, AXIS_Y);
}
void Application::Update(void)
{
	//Update the system so it knows how much time has passed since the last call
	m_pSystem->Update();

	//Is the arcball active?
	ArcBall();

	//Is the first person camera active?
	CameraRotation();
}
void Application::Display(void)
{
	// Clear the screen
	ClearScreen();

	static float fAngle = 0.0f;

	//camera projection
	matrix4 m4Projection = m_pCameraMngr->GetProjectionMatrix();
	matrix4 m4View = m_pCameraMngr->GetViewMatrix();
	matrix4 m4Model = ToMatrix4(m_qArcBall);

	//Setting up everything for a timer
	static float fTimer = 0;
	static uint uClock = m_pSystem->GenClock();
	fTimer += static_cast<float>(m_pSystem->GetDeltaTime(uClock));

	//Rotating and translating the diamonds as needed
	m4Model = glm::rotate(m4Model, glm::radians(fAngle), vector3(0.0f, 0.0f, 1.0f));

	for (int i = 0; i < m_uMeshCount; i++)
	{
		m_pMeshList[i]->Render(m4Projection, m4View, glm::translate(m4Model, vector3(2.0f * a_fRadius, 0.0f, 0.0f)));
	}

	m4Model = glm::translate(m4Model, vector3(2.0f * a_fRadius, 0.0f, 0.0f));

	for (int i = 0; i < m_uMeshCount; i++)
	{
		m_pMeshList[i]->Render(m4Projection, m4View, glm::rotate(m4Model, glm::radians(fAngle), vector3(0.0f, 1.0f, 0.0f)));
	}

	//Checking when each second has passed and modifying the correct variables
	if (fTimer >= 1.0f)
	{
		fTimer = 0.0f;
		a_fRadius += 1.0f;
		MyMesh* tempMesh = new MyMesh();
		tempMesh->GenerateDiamond(m_uSides, 5.0f, 3.0f, C_GREEN);
		m_pMeshList.push_back(tempMesh);
		m_uMeshCount++;
		m_uSides++;
	}

	// draw a skybox
	m_pMeshMngr->AddSkyboxToRenderList();
	
	//render list call
	m_uRenderCallCount = m_pMeshMngr->Render();

	//clear the render list
	m_pMeshMngr->ClearRenderList();
	
	//draw gui
	DrawGUI();

	//Angle increment
	fAngle += 1.0f;
	
	//end the current frame (internally swaps the front and back buffers)
	m_pWindow->display();
}
void Application::Release(void)
{
	std::vector<MyMesh*> m_pMeshList;
	uint uSize = m_pMeshList.size();
	for (uint i = 0; i < uSize; ++i)
	{
		if (m_pMeshList[i] != nullptr)
		{
			delete m_pMeshList[i];
			m_pMeshList[i] = nullptr;
		}
	}
	//release GUI
	ShutdownGUI();
}