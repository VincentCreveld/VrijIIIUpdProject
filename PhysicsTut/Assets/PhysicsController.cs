using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class PhysicsController : MonoBehaviour
{
	private Rigidbody rb;
	
	public Transform centerRay;
	private float rayLength = 0.2f;
	public LayerMask rayMask = 0;

	private Vector3 normalVector = Vector3.zero;
	private bool isGrounded = false;

	// zwaartekracht
	private Vector3 gravityVector = Vector3.zero;
	// overige beweging
	private Vector3 movementVector = Vector3.zero;

	private float uK = 0.1f; // kinetische weerstand sneeuw
	private float uS = 0.14f;// statische weerstand sneeuw
	private float C = 0.5f;  // wrijvings coefficient
	private float p = 1.21f; // dichtheid van lucht
	private float A = 0.72f; // oppervlakte voorwerp (0.72m2 is de oppervlakte van een gemiddeld staand mens.)
	private float m = 70f;	 // massa
	private float g = 9.81f; // zwaartekracht aarde
	private bool isKinetic;

	private void Awake()
	{
		rb = GetComponent<Rigidbody>();
	}

	private void FixedUpdate()
	{
		SetIsGrounded();

		CalculatePhysicsValues();
	}

	// Berekent de beweging voor de massa aan de hand van natuurkundige formules.
	// Na de berekening worden de krachten aan de massa toegevoegd.
	private void CalculatePhysicsValues()
	{
		Vector3 curVelocity = rb.velocity;
		Vector3 velocitySum = Vector3.zero;
		Vector3 airResistVec = Vector3.zero;
		Vector3 gravityVelocity = Vector3.zero;

		Vector3 slopeDirection = rb.transform.forward + normalVector;

		float angle = Mathf.Acos(Vector3.Dot(Vector3.up.normalized, normalVector));

		float Fg = m * g;
		float FgPar = Fg * (Mathf.Sin(angle));
		float FgPer = Mathf.Abs(Fg * (Mathf.Cos(angle)));
		float Ffric = GetForceFriction(FgPar, FgPer);
		float Fres = FgPar - Ffric;

		float a = Fres / m;
		float v = curVelocity.magnitude + (a * Time.fixedDeltaTime);
		float Fair = 0.5f * (C * p * A * Mathf.Pow(v, 2));

		gravityVelocity = -Vector3.up * (m * g * Time.fixedDeltaTime);


		velocitySum = curVelocity;
		if(isGrounded)
			velocitySum += slopeDirection.normalized * (Fres / m * Time.fixedDeltaTime);

		airResistVec = (velocitySum.normalized * -1) * (Fair / m * Time.fixedDeltaTime);

		velocitySum += airResistVec;

		rb.velocity = velocitySum;
		rb.AddForce(gravityVelocity, ForceMode.Acceleration);
	}

	// Schiet een raycast naar beneden vanuit de massa en kijkt of het in contact is met een schans
	private void SetIsGrounded()
	{
		Ray ray = new Ray(centerRay.position, -Vector3.up);
		RaycastHit hit;
		if(Physics.Raycast(ray, out hit, rayLength, rayMask))
		{
			normalVector = hit.normal.normalized;
			isGrounded = true;
		}
		else
		{
			normalVector = Vector3.up;
			isGrounded = false;
		}
	}

	// Geeft een weerstand aan de hand van de bestaande beweging.
	private float GetForceFriction(float forwardMagnitude, float gravity)
	{
		float Ff = 0;

		if(Mathf.Abs(forwardMagnitude) > uS * gravity)
		{
			Ff = uK * gravity;
			isKinetic = true;
		}
		else
		{
			Ff = forwardMagnitude;
			isKinetic = false;
		}

		return Ff;
	}
}
