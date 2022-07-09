using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PointRotation : MonoBehaviour
{
    [Header("Distance Method")]
    [SerializeField] Transform pointA;
    [SerializeField] Transform pointB;
    [SerializeField] Transform pointC;
    [SerializeField][Range(0f,5f)] float y2;
    [SerializeField][Range(0f, 5f)] float y3;

    [Space]
    [Header("Trigo Method")]
    [SerializeField][Range(0, 5)] float input1;
    [SerializeField][Range(0, 5)] float input2;

    const float TAU = 6.28318530718f;

    float distanceAB;
    float distanceBC;
    float x1;
    float x2;
    float x3;
    float y1 = 5f;
    private void Start()
    {
        //distanceAB = Vector3.Distance(pointA.position, pointB.position);
        distanceAB = 5f;
        distanceBC = 5f;
        pointA.position = new Vector2(0, y1);
        x1 = pointA.position.x;
    }

    private void Update()
    {
        TrigMethod(0, 5, pointB, distanceAB, input1);
        TrigMethod(0, 5, pointC, distanceBC, input2);
    }

    private void DistanceMethod()
    {
        x2 = x1 + Mathf.Abs(Mathf.Sqrt(distanceAB * distanceAB - Mathf.Pow(y2 - y1, 2)));

        x3 = x2 + Mathf.Abs(Mathf.Sqrt(distanceBC * distanceBC - Mathf.Pow(y3 - y2, 2)));

        pointB.localPosition = new Vector3(x2, y2);
        pointC.localPosition = new Vector2(x3 ,y3);
    }

    private void TrigMethod(float y1, float y2,Transform targetTransform, float distance, float input)
    {
        float angleInRadians = RangeInterpolate(y2, y1, TAU / 2 - (180 * Mathf.Deg2Rad), TAU / 4 - (180 * Mathf.Deg2Rad), input);
        Vector2 position = AngToDir(angleInRadians) * distance;
        targetTransform.localPosition = position;
    }

    Vector2 AngToDir(float angInRads)
    {
        return new Vector2(Mathf.Cos(angInRads), Mathf.Sin(angInRads));
    }

    float RangeInterpolate(float a, float b, float c, float d, float input)
    {
        return c + ((d - c) / (b - a)) * (input - a);
    }
}
