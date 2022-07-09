using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public class HandPresenceCustom : MonoBehaviour
{
    public GameObject handPrefab = null;
    public GameObject handSpawnGameObject = null;
    public InputDeviceCharacteristics deviceCharacteristics;
    Animator anim;

    InputDevice targetDevice;

    private void Start()
    {
        TryInitialize();
    }

    private void Update()
    {
        if (!targetDevice.isValid)
        {
            TryInitialize();
        }
        UpdateHandAnimations();
    }

    void UpdateHandAnimations()
    {
        if(targetDevice.TryGetFeatureValue(CommonUsages.trigger, out float triggerValue))
        {
            anim.SetFloat("Trigger", triggerValue);
        }
        else
        {
            anim.SetFloat("Trigger", 0);
        }

        if(targetDevice.TryGetFeatureValue(CommonUsages.grip, out float gripValue))
        {
            anim.SetFloat("Grip", gripValue);
        }
        else
        {
            anim.SetFloat("Grip", 0);
        }
    }

    private void TryInitialize()
    {
        List<InputDevice> inputDevices = new List<InputDevice>();
        InputDevices.GetDevicesWithCharacteristics(deviceCharacteristics, inputDevices);
        handSpawnGameObject = Instantiate(handPrefab, transform);
        anim = handSpawnGameObject.GetComponent<Animator>();

        if (inputDevices.Count > 0)
            targetDevice = inputDevices[0];
    }
}
