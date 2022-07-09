using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PIDObject : MonoBehaviour
{
    [SerializeField] Rigidbody playerRigidBody;
    [SerializeField] float frequency1 = 50f;
    [SerializeField] float damping1 = 1f;
    [SerializeField] float frequency2 = 50f;
    [SerializeField] float damping2 = 1f;

    [SerializeField] float rotFrequency = 100f;
    [SerializeField] float rotDamping = 0.9f;

    [Header("Target")]
    [SerializeField] Transform target1;
    [SerializeField] Transform target2;

    [SerializeField] Transform forceTransform1 = null;
    [SerializeField] Transform forceTransform2 = null;
    [SerializeField] Rigidbody myRigidbody;

    private void Awake()
    {
    }

    private void Start()
    {
        transform.SetPositionAndRotation(target1.position, target1.rotation);
        //myRigidbody.centerOfMass = myRigidbody.position + transform.TransformDirection(transform.forward * 1.2f);
        myRigidbody.maxAngularVelocity = float.PositiveInfinity;
    }

    private void FixedUpdate()
    {
        PIDMovement(target1, forceTransform1, frequency1, damping1);
        //PIDMovement(target2, forceTransform2, frequency2, damping2);
        //PIDRotation();
    }

    void PIDMovement(Transform target,Transform forceTransform,float frequency, float damping)
    {
        float kp = (6f * frequency) * (6f * frequency) * 0.25f;
        float kd = 4.5f * frequency * damping;
        float g = 1 / (1 + kd * Time.fixedDeltaTime + kp * Time.fixedDeltaTime * Time.fixedDeltaTime);
        float ksg = kp * g;
        float kdg = (kd + kp * Time.fixedDeltaTime) * g;
        Vector3 force = (target.position - transform.position) * ksg + (playerRigidBody.velocity - myRigidbody.velocity) * kdg;
        //Vector3 force = (target.position - transform.position) * ksg + (playerRigidBody.velocity - myRigidbody.velocity) * kdg;
        //myRigidbody.AddForce(force, ForceMode.Acceleration);
        myRigidbody.AddForceAtPosition(force, forceTransform.position, ForceMode.Acceleration);
    }

    //void PIDRotation()
    //{
    //    float kp = (6f * rotFrequency) * (6f * rotFrequency) * 0.25f;
    //    float kd = 4.5f * rotFrequency * rotDamping;
    //    float g = 1 / (1 + kd * Time.fixedDeltaTime + kp * Time.fixedDeltaTime * Time.fixedDeltaTime);
    //    float ksg = kp * g;
    //    float kdg = (kd + kp * Time.fixedDeltaTime) * g;
    //    Quaternion q = target.rotation * Quaternion.Inverse(transform.rotation);
    //    if (q.w < 0)
    //    {
    //        q.x = -q.x;
    //        q.y = -q.y;
    //        q.z = -q.z;
    //        q.w = -q.w;
    //    }

    //    q.ToAngleAxis(out float angle, out Vector3 axis);
    //    axis.Normalize();
    //    axis *= Mathf.Deg2Rad;
    //    Vector3 torque = ksg * axis * angle + -myRigidbody.angularVelocity * kdg;
    //    myRigidbody.AddTorque(torque, ForceMode.Acceleration);
    //}

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        var vec = myRigidbody.position + myRigidbody.rotation * myRigidbody.centerOfMass;
        Gizmos.DrawSphere(vec, 0.12f);
    }
}
