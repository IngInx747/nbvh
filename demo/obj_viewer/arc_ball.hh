#ifndef ARC_BALL_H
#define ARC_BALL_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

class ArcBall
{
public:
	ArcBall() :
		m_center(),
		m_radius(1.0f)
	{}

	ArcBall(float r, float x, float y)
	{
		set(r, x, y);
	}

	void set(float r, float x, float y)
	{
		m_radius = r;
		m_center = _coord2sphere(x, y);
	}

	glm::quat update_quat(float x, float y)
	{
		glm::vec3 v = _coord2sphere(x, y);
		glm::vec3 p = glm::cross(m_center, v);
		glm::quat q(glm::dot(m_center, v), p[0], p[1], p[2]);
		m_center = v;
		return q;
	}

	glm::vec3& center() { return m_center; }
	const glm::vec3& center() const { return m_center; }

	float& radius() { return m_radius; }
	const float& radius() const { return m_radius; }

protected:
	// map planer coordinates to vector of unit sphere
	inline glm::vec3 _coord2sphere(float x, float y)
	{
		glm::vec2 uv(x, y);
		uv /= m_radius;
		float l = glm::length(uv);

		if (l > 1) return glm::vec3(uv / l, 0);
		else return glm::vec3(uv, std::sqrtf(1 - l * l));
	}

protected:
	// center of sphere
	glm::vec3 m_center;

	// radius of sphere
	float m_radius = 1.0f;
};

#endif // !ARC_BALL_H
