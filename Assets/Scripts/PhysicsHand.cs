using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class PhysicsHand : MonoBehaviour
{
    [SerializeField] Rigidbody playerRigidBody;
    [SerializeField] float frequency = 50f;
    [SerializeField] float damping = 1f;

    [SerializeField] float rotFrequency = 100f;
    [SerializeField] float rotDamping = 0.9f;

    [Header("Target")]
    [SerializeField] Transform target;
    [SerializeField] Renderer nonPhysicalHandRenderer;
    [SerializeField] float nonPhysicalHandShowDistance = 0.05f;

    [Space]
    [Header("Springs")]
    [SerializeField] float climbForce = 1000f;
    [SerializeField] float climbDrag = 500f;

    [Header("Interactions")]
    [SerializeField] InputActionReference selectInputActionReference = null;

    Vector3 previousPosition = Vector3.zero;
    Rigidbody rb = null;
    Rigidbody interatableRb = null;
    SpringJoint springJoint = null;

    bool isCollidingWithInteractable = false;

    private void OnEnable()
    {
        selectInputActionReference.action.started += InitiateSpringJoint;
    }

    private void OnDisable()
    {
        selectInputActionReference.action.canceled -= InitiateSpringJoint;
    }

    private void Awake()
    {
        springJoint = GetComponent<SpringJoint>();
    }

    private void Start()
    {
        transform.position = target.position;
        transform.rotation = target.rotation;
        previousPosition = transform.position;

        rb = GetComponent<Rigidbody>();
        rb.maxAngularVelocity = float.PositiveInfinity;
    }

    private void Update()
    {
        //ActivateTrackedHand();
    }

    private void FixedUpdate()
    {
        //PIDMovement();

        //PIDRotation();

        PhysicsMovement();
        PhysicsRotation();

        //if(isColliding)
        //    HooksLaw();
    }

    private void InitiateSpringJoint(InputAction.CallbackContext obj)
    {
        if (isCollidingWithInteractable && obj.control.IsPressed())
            springJoint.connectedBody = interatableRb;
        //else
        //    springJoint.connectedBody = null;
    }


    void PIDMovement()
    {
        float kp = (6f * frequency) * (6f * frequency) * 0.25f;
        float kd = 4.5f * frequency * damping;
        float g = 1 / (1 + kd * Time.fixedDeltaTime + kp * Time.fixedDeltaTime * Time.fixedDeltaTime);
        float ksg = kp * g;
        float kdg = (kd + kp * Time.fixedDeltaTime) * g;
        Vector3 force = (target.position - transform.position) * ksg + (playerRigidBody.velocity - rb.velocity) * kdg;
        rb.AddForce(force, ForceMode.Acceleration);
    }

    void PIDRotation()
    {
        float kp = (6f * rotFrequency) * (6f * rotFrequency) * 0.25f;
        float kd = 4.5f * rotFrequency * rotDamping;
        float g = 1 / (1 + kd * Time.fixedDeltaTime + kp * Time.fixedDeltaTime * Time.fixedDeltaTime);
        float ksg = kp * g;
        float kdg = (kd + kp * Time.fixedDeltaTime) * g;
        Quaternion q = target.rotation * Quaternion.Inverse(transform.rotation);
        if(q.w < 0)
        {
            q.x = -q.x;
            q.y = -q.y;
            q.z = -q.z;
            q.w = -q.w;
        }

        q.ToAngleAxis(out float angle, out Vector3 axis);
        axis.Normalize();
        axis *= Mathf.Deg2Rad;
        Vector3 torque = ksg * axis * angle + -rb.angularVelocity * kdg;
        rb.AddTorque(torque, ForceMode.Acceleration);
    }

    void PhysicsMovement()
    {
        rb.velocity = (target.position - transform.position) / Time.fixedDeltaTime;
    }

    void PhysicsRotation()
    {
        Quaternion rotationDifference = target.rotation * Quaternion.Inverse(transform.rotation);
        rotationDifference.ToAngleAxis(out float angleInDegree, out Vector3 rotationAxis);
        Vector3 rotationDifferenceInDegree = angleInDegree * rotationAxis;

        rb.angularVelocity = (rotationDifferenceInDegree * Mathf.Deg2Rad) / Time.fixedDeltaTime;
    }

    void HooksLaw()
    {
        Vector3 displacementFromResting = transform.position - target.position;
        Vector3 force = displacementFromResting * climbForce;
        
        float drag = GetDrag();
        playerRigidBody.AddForce(force, ForceMode.Acceleration);
        playerRigidBody.AddForce(drag * -playerRigidBody.velocity * climbDrag, ForceMode.Acceleration);
    }

    float GetDrag()
    {
        Vector3 handVelocity = (target.localPosition - previousPosition) / Time.fixedDeltaTime;
        float drag = 1 / handVelocity.magnitude + 0.01f;
        drag = (drag > 1) ? 1 : drag;
        drag = (drag < 0.03) ? 0.03f : drag;
        previousPosition = transform.position;
        return drag;
    }

    private void OnCollisionEnter(Collision collision)
    {
    }

    private void OnCollisionExit(Collision collision)
    {
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Interactable"))
        {
            isCollidingWithInteractable = true;
            interatableRb = other.GetComponent<Rigidbody>();
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Interactable"))
        {
            isCollidingWithInteractable = false;
            interatableRb = null;
        }
    }

    private void ActivateTrackedHand()
    {
        if (Vector3.Distance(transform.position, target.position) > nonPhysicalHandShowDistance)
        {
            nonPhysicalHandRenderer.enabled = true;
        }
        else
        {
            nonPhysicalHandRenderer.enabled = false;
        }
    }
}
