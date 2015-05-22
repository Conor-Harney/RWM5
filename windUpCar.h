#ifndef WINDUPCAR_H
#define WINDUPCAR_H

class windUpCar : public Test
{
public:
	float angle;
	b2Body* keyBody;
	b2Body* circleBody;
	b2Vec2 pos;
	float radians;
	b2WeldJointDef weldJointDef1;
	windUpCar()
	{
		radians = 0;
		angle = 0;
		//CreateElasticRope();
		//createBlock();
		//createCantilever();
		createBridge();
	}
	void CreateElasticRope() {
		//=======Params
		// Position and size
		b2Vec2 lastPos = b2Vec2(4,4); //set position first body
		float widthBody = 0.35;
		float heightBody = 0.1;
		// Body params
		float density = 0.05;
		float restitution = 0.5;
		float friction = 0.5;
		// Distance joint
		float dampingRatio = 0.0;
		float frequencyHz = 0;
		// Rope joint
		float kMaxWidth = 1.1;
		// Bodies
		int countBodyInChain = 15;
		b2Body* prevBody;

		//========Create bodies and joints
		for (int k = 0; k < countBodyInChain; k++) {
			b2BodyDef bodyDef;
			if(k==0 ) bodyDef.type = b2_staticBody; //first body is static
			else bodyDef.type = b2_dynamicBody;
			bodyDef.position = lastPos;
			lastPos += b2Vec2(0, -2*widthBody); //modify b2Vect for next body
			bodyDef.fixedRotation = true;
			b2Body* body = m_world->CreateBody(&bodyDef);

			b2PolygonShape distBodyBox; 
			distBodyBox.SetAsBox(widthBody, heightBody);
			b2FixtureDef fixDef;
			fixDef.density = density;
			fixDef.restitution = restitution;
			fixDef.friction = friction;
			fixDef.shape = &distBodyBox;
			body->CreateFixture(&fixDef);
			//body->SetHealth(9999999);
			body->SetLinearDamping(0.0005f);

			if(k>0) {
				//Create distance joint
				b2DistanceJointDef distJDef;
				b2Vec2 anchor1 = prevBody->GetWorldCenter();
				b2Vec2 anchor2 = body->GetWorldCenter();
				distJDef.Initialize(prevBody, body, anchor1, anchor2);
				distJDef.collideConnected = false;
				distJDef.dampingRatio = dampingRatio;
				distJDef.frequencyHz = frequencyHz;
				m_world->CreateJoint(&distJDef);

				//Create rope joint
				b2RopeJointDef rDef;
				rDef.maxLength = (body->GetPosition() - prevBody->GetPosition()).Length() * kMaxWidth;
				rDef.localAnchorA = rDef.localAnchorB = b2Vec2_zero;
				rDef.bodyA = prevBody;
				rDef.bodyB = body;
				m_world->CreateJoint(&rDef);

			} //if k>0
			prevBody = body;
		}
	}
	void createBlock(){
		b2Vec2 lastPos = b2Vec2(3,6); //set position first body
		float widthBody = 1.0;
		float heightBody = 1.0;
		// Body params
		float density = 0.05;
		float restitution = 0.5;
		float friction = 0.5;
		// Distance joint
		float dampingRatio = 0.0;
		float frequencyHz = 0;
		// Rope joint
		float kMaxWidth = 1.1;
		// Bodies
		int countBodyInChain = 15;
		b2Body* prevBody;
		b2BodyDef bodyDef;
		bodyDef.type = b2_staticBody; //first body is static
		bodyDef.position = lastPos;	
		bodyDef.fixedRotation = true;
		b2Body* body = m_world->CreateBody(&bodyDef);
		b2PolygonShape distBodyBox; 
		distBodyBox.SetAsBox(widthBody, heightBody);
		b2FixtureDef fixDef;
		fixDef.density = density;
		fixDef.restitution = restitution;
		fixDef.friction = friction;
		fixDef.shape = &distBodyBox;
		body->CreateFixture(&fixDef);
		//body->SetHealth(9999999);
		body->SetLinearDamping(0.0005f);
		prevBody = body;
	}

	void createCantilever()
	{

		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(4.0f, 0.125f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 20.0f;

			b2WeldJointDef jd;
			jd.frequencyHz = 5.0f;
			jd.dampingRatio = 0.7f;

			b2Body* prevBody = ground;
			
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(-14.0f + 2.0f, 15.0f);
				keyBody = m_world->CreateBody(&bd);
				keyBody->CreateFixture(&fd);

				b2Vec2 anchor(-15.0f + 2.0f, 15.0f);
				jd.Initialize(prevBody, keyBody, anchor);
				m_world->CreateJoint(&jd);

				prevBody = keyBody;
		}
	}

	void createBridge()
	{
		b2Vec2 circleAnchorPoint(0,0);
		b2Body* lastLink;
		b2Body* m_middle;
		float linkSize = 0.1;
		enum
		{
			e_count = 50
		};

		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(linkSize, linkSize);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 20.0f;
			fd.friction = 0.2f;

			
			

			b2Body* prevBody = ground;
			for (int32 i = 0; i < e_count; ++i)
			{
				//bridge points
				b2BodyDef bd;
				if(i == 0)
				{
					bd.type = b2_staticBody;
				}
				else
				{
					bd.type = b2_dynamicBody;
				}
				bd.position.Set(-14.5f + ((linkSize * 6) * i), 5.0f);
				circleAnchorPoint = bd.position;
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);

				b2Vec2 anchor(bd.position);
				//if(i == 3)
				//{
					b2DistanceJointDef dj;
					dj.Initialize(prevBody, body, prevBody->GetWorldCenter(), body->GetWorldCenter());
					dj.length = 0.3;
					if(i < e_count / 3)
					{
						dj.frequencyHz = 5;
					}
					else
					{
						dj.frequencyHz = 30.0;
					}
					m_world->CreateJoint(&dj);
				/*}
				else if(i == e_count)
				{
					b2RevoluteJointDef rj;
					rj.Initialize(prevBody, body, body->GetPosition());
					rj.collideConnected = false;
					m_world->CreateJoint(&rj);

				}
				else
				{
					b2RevoluteJointDef rj;
					rj.Initialize(prevBody, body, body->GetPosition());
					rj.collideConnected = false;
					m_world->CreateJoint(&rj);
					
				}*/

				if (i == (e_count >> 1))
				{
					m_middle = body;
				}
				prevBody = body;
			}
			lastLink = prevBody;
			
			b2Vec2 anchor(circleAnchorPoint);
			//jd.Initialize(prevBody, ground, anchor);
			//m_world->CreateJoint(&jd);
		}
		{
			//circle
			b2CircleShape shape;
			shape.m_radius = 1.5f;

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;

			b2BodyDef bd;
			bd.type = b2_kinematicBody;
			pos = b2Vec2(circleAnchorPoint.x, circleAnchorPoint.y);
			pos.x += shape.m_radius;
			bd.position.Set(pos.x, pos.y);
			
			circleBody = m_world->CreateBody(&bd);
			circleBody->CreateFixture(&fd);
			
			weldJointDef1.Initialize(circleBody, lastLink, circleBody->GetLocalCenter());
			m_world->CreateJoint(&weldJointDef1);
			circleBody->SetTransform(pos,radians);
		}
	}
	void Step(Settings* settings)
	{
		Test::Step(settings);
		m_debugDraw.DrawString(5, m_textLine, "Radians = %4.2f" , radians);
		m_textLine += DRAW_STRING_NEW_LINE;
		m_debugDraw.DrawString(5, m_textLine, "Degrees =%4.2f ", angle);
	}

	void Keyboard(unsigned char key)
	{
		pos = circleBody->GetPosition();
		switch (key)
		{
			case 'i':
				angle += 0.0001;
				break;
			case 'k':
				angle -= 0.0001;
				break;
			
		}
		if(key == 'w'){
			m_world->DestroyBody(circleBody);
		}
		if(key == 'a'){
			circleBody->SetLinearVelocity(b2Vec2(-1,0));
		}
		if(key == 's'){
			circleBody->SetAngularVelocity(0.5);
		}
		else if( key == 'd')
		{
			circleBody->SetLinearVelocity(b2Vec2(1,0));
		}
		else
		{
			circleBody->SetAngularVelocity(0);
			circleBody->SetLinearVelocity(b2Vec2(0,0));
		}
		radians =  angle * 3.1459f / 180;
		keyBody->SetTransform( keyBody->GetPosition(),radians);
		circleBody->SetTransform(pos,radians);
		
	}
	static Test* Create()
	{
		return new windUpCar();
	}
};
	
#endif