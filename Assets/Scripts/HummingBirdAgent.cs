using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using System;
using Unity.MLAgents.Sensors;

public class HummingBirdAgent : Agent
{
    public float moveForce = 2f;

    public float pitchSpeed = 100f;

    public float yawSpeed = 100f;

    public Transform beakTip;

    public Camera agentCamera;

    public bool trainingmode;

    new private Rigidbody rigidbody;

    private FlowerArea FlowerArea;

    private Flower nearestFlower;

    private float smoothPitchSpeed = 0f;

    private float smoothYawSpeed = 0f;

    private const float maxPitchAngle = 80f;

    private const float beakTipRadius = 0.008f;

    private bool frozen = false;

    public float nectarObtained { get; private set; }

    public override void Initialize()
    {
        rigidbody = GetComponent<Rigidbody>();
        FlowerArea = GetComponentInParent<FlowerArea>();

        if (!trainingmode) MaxStep = 0;
    }

    public override void OnEpisodeBegin()
    {
        if (trainingmode)
        {
            FlowerArea.ResetFlowers();
        }

        nectarObtained = 0f;

        rigidbody.velocity = Vector3.zero;
        rigidbody.angularVelocity = Vector3.zero;

        bool InFrontOfFlower = true;
        if (trainingmode)
        {
            InFrontOfFlower = UnityEngine.Random.value > 0.5f;
        }

        MoveToSafeRandomPos(InFrontOfFlower);

        UpdateNearestFlower();
    }

    public override void OnActionReceived(float[] vectorAction)
    {
        if (frozen) return;

        Vector3 move = new Vector3(vectorAction[0], vectorAction[1], vectorAction[2]);

        rigidbody.AddForce(move * moveForce);

        Vector3 rotationVector = transform.rotation.eulerAngles;

        float pitchChange = vectorAction[3];
        float yawChange = vectorAction[4];

        smoothPitchSpeed = Mathf.MoveTowards(smoothPitchSpeed, pitchChange, 2f * Time.fixedDeltaTime);
        smoothYawSpeed = Mathf.MoveTowards(smoothYawSpeed, yawChange, 2f * Time.fixedDeltaTime);

        float pitch = rotationVector.x + smoothPitchSpeed * Time.fixedDeltaTime * pitchSpeed;
        if (pitch > 180f) pitch -= 360f;
        pitch = Mathf.Clamp(pitch, -maxPitchAngle, maxPitchAngle);

        float yaw = rotationVector.y + smoothYawSpeed * Time.fixedDeltaTime * yawSpeed;

        transform.rotation = Quaternion.Euler(pitch, yaw, 0f);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if(nearestFlower == null)
        {
            sensor.AddObservation(new float[10]);
            return;
        }
        sensor.AddObservation(transform.localRotation.normalized);

        Vector3 toFlower = nearestFlower.flowerCenterPosition - beakTip.position;

        sensor.AddObservation(toFlower.normalized);

        sensor.AddObservation(Vector3.Dot(toFlower.normalized, -nearestFlower.flowerUpVector.normalized));

        sensor.AddObservation(Vector3.Dot(beakTip.forward.normalized, -nearestFlower.flowerUpVector.normalized));

        sensor.AddObservation(toFlower.magnitude / FlowerArea.areaDiameter);
;    }

    public override void Heuristic(float[] actionsOut)
    {
        Vector3 forward = Vector3.zero;
        Vector3 left = Vector3.zero;
        Vector3 up = Vector3.zero;
        float pitch = 0f;
        float yaw = 0f;

        if (Input.GetKey(KeyCode.W))
        {
            forward = transform.forward;
            Debug.Log("yes");
        }
        else if (Input.GetKey(KeyCode.S)) forward = -transform.forward;

        if (Input.GetKey(KeyCode.A)) left = -transform.right;
        else if (Input.GetKey(KeyCode.D)) left = transform.right;

        if (Input.GetKey(KeyCode.E)) up = transform.up;
        else if (Input.GetKey(KeyCode.C)) up = -transform.up;

        if (Input.GetKey(KeyCode.UpArrow)) pitch = -1f;
        else if (Input.GetKey(KeyCode.DownArrow)) pitch = 1f;

        if (Input.GetKey(KeyCode.LeftArrow)) yaw = -1f;
        else if (Input.GetKey(KeyCode.RightArrow)) yaw = 1f;

        Vector3 combined = (forward + left + up).normalized;

        actionsOut[0] = combined.x;
        actionsOut[1] = combined.y;
        actionsOut[2] = combined.z;
        actionsOut[3] = pitch;
        actionsOut[4] = yaw;
    }

    public void FreezeAgent()
    {
        Debug.Assert(trainingmode == false, "training mode on");
        frozen = true;
        rigidbody.Sleep();
    }

    public void UnFreezeAgent()
    {
        Debug.Assert(trainingmode == false, "training mode on");
        frozen = false;
        rigidbody.WakeUp();
    }

    private void MoveToSafeRandomPos(bool inFrontOfFlower)
    {
        bool safePosFound = false;
        int attemptsRemaining = 100;

        Vector3 potentialPos = Vector3.zero;
        Quaternion potentialRot = new Quaternion();

        while(!safePosFound && attemptsRemaining > 0)
        {
            attemptsRemaining--;
            if (inFrontOfFlower)
            {
                Flower randomFlower = FlowerArea.flowers[UnityEngine.Random.Range(0, FlowerArea.flowers.Count)];
                float disFromFlower = UnityEngine.Random.Range(0.1f, 0.2f);
                potentialPos = randomFlower.transform.position + disFromFlower * randomFlower.flowerUpVector;
                Vector3 toFlower = randomFlower.flowerCenterPosition - potentialPos;
                potentialRot = Quaternion.LookRotation(toFlower, Vector3.up);
            }
            else
            {
                float height = UnityEngine.Random.Range(1.2f, 2.5f);
                float radius = UnityEngine.Random.Range(2f, 7f);
                Quaternion direction = Quaternion.Euler(0f, UnityEngine.Random.Range(-180f, 180f), 0f);
                potentialPos = FlowerArea.transform.position + Vector3.up * height + direction * Vector3.forward * radius;

                float pitch = UnityEngine.Random.Range(-60f, 60f);
                float yaw = UnityEngine.Random.Range(-180f, 180f);
                potentialRot = Quaternion.Euler(pitch, yaw, 0f);
            }

            Collider[] colliders = Physics.OverlapSphere(potentialPos, 0.05f);

            safePosFound = colliders.Length == 0;
        }

        Debug.Assert(safePosFound, "could not find safe pos");
        transform.position = potentialPos;
        transform.rotation = potentialRot;
    } 

    private void UpdateNearestFlower()
    {
        foreach(Flower flower in FlowerArea.flowers)
        {
            if(nearestFlower == null && flower.HasNectar)
            {
                nearestFlower = flower;
            }else if (flower.HasNectar)
            {
                float disToFlower = Vector3.Distance(flower.transform.position, beakTip.position);
                float disToNearestFlower = Vector3.Distance(nearestFlower.transform.position, beakTip.position);

                if(!nearestFlower.HasNectar || disToFlower < disToNearestFlower)
                {
                    nearestFlower = flower;
                }
            }
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        TriggerEnterOrStay(other);
    }

    private void OnTriggerStay(Collider other)
    {
        TriggerEnterOrStay(other);
    }

    private void TriggerEnterOrStay(Collider collider)
    {
        if (collider.CompareTag("nectar"))
        {
            Vector3 closestPoint = collider.ClosestPoint(beakTip.position);

            if(Vector3.Distance(beakTip.position, closestPoint) < beakTipRadius)
            {
                Flower flower = FlowerArea.GetFlowerFromNectar(collider);

                float nectarRecieved = flower.feed(0.01f);

                nectarObtained += nectarRecieved;

                if (trainingmode)
                {
                    float bonus = 0.02f * Mathf.Clamp01(Vector3.Dot(transform.forward.normalized, -nearestFlower.flowerUpVector.normalized));
                    AddReward(0.01f + bonus);

                    if (!flower.HasNectar)
                    {
                        UpdateNearestFlower();
                    }
                }
            }
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (trainingmode && collision.collider.CompareTag("boundary"))
        {
            AddReward(-0.5f);
        }
    }

    private void Update()
    {
        if(nearestFlower != null)
        {
            Debug.DrawLine(beakTip.position, nearestFlower.flowerCenterPosition, Color.green);
        }
    }

    private void FixedUpdate()
    {
        if(nearestFlower != null && !nearestFlower.HasNectar)
        {
            UpdateNearestFlower();
        }
    }
}
