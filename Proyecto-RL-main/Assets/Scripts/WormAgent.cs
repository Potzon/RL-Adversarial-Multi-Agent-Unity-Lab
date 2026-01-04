using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;

[RequireComponent(typeof(JointDriveController))]
public class WormAgent : Agent
{
    const float m_MaxSpeed = 10;

    private float previousDist;

    [Header("Predator reference")]
    public Transform predator;   // <<< EL DEPREDADOR


    [Header("Body Parts")]
    public Transform bodySegment0;
    public Transform bodySegment1;
    public Transform bodySegment2;
    public Transform bodySegment3;

    private Transform m_TargetTemp;

    OrientationCubeController m_OrientationCube;
    JointDriveController m_JdController;

    DirectionIndicator m_DirectionIndicator;

    Vector3 startPos;

    public override void Initialize()
    {
        startPos = bodySegment0.position;

        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_JdController = GetComponent<JointDriveController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();

        UpdateOrientationObjects();

        m_JdController.SetupBodyPart(bodySegment0);
        m_JdController.SetupBodyPart(bodySegment1);
        m_JdController.SetupBodyPart(bodySegment2);
        m_JdController.SetupBodyPart(bodySegment3);
    }

    public override void OnEpisodeBegin()
    {
        foreach (var bp in m_JdController.bodyPartsList)
            bp.Reset(bp);

        /* bodySegment0.rotation = Quaternion.Euler(0, Random.Range(0, 360f), 0);
        bodySegment0.position = GetRandomPos(); */

        UpdateOrientationObjects();
        previousDist = Vector3.Distance(bodySegment0.position, predator.position);

    }

    Vector3 GetRandomPos()
    {
        Vector3 predatorBodyPos = predator.GetComponent<CrawlerAgent>().body.position;
        Vector3 pos;
        do
        {
            pos = new Vector3(
                Random.Range(-15f, 15f),
                1.15f,
                Random.Range(-15f, 15f)
            );
        }
        while (Vector3.Distance(pos, predatorBodyPos) < 14f);

        return pos;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Distancia al suelo
        if (Physics.Raycast(bodySegment0.position, Vector3.down, out var hit, 10))
            sensor.AddObservation(hit.distance / 10f);
        else
            sensor.AddObservation(1);

        // Vector de escape (worm - predator)
        Vector3 escapeVector = bodySegment0.position - predator.position;
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(escapeVector.normalized));
        sensor.AddObservation(escapeVector.magnitude / 50f);  // distancia normalizada

        // Velocidad del worm  
        sensor.AddObservation(
            m_OrientationCube.transform.InverseTransformDirection(
                m_JdController.bodyPartsDict[bodySegment0].rb.linearVelocity
            )
        );

        // Observaciones de cada parte
        foreach (var bp in m_JdController.bodyPartsList)
            CollectObservationBodyPart(bp, sensor);
    }

    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        sensor.AddObservation(bp.groundContact.touchingGround ? 1 : 0);
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.linearVelocity));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.angularVelocity));

        if (bp.joint != null)
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var bp = m_JdController.bodyPartsDict;

        int i = -1;
        var a = actions.ContinuousActions;

        bp[bodySegment0].SetJointTargetRotation(a[++i], a[++i], 0);
        bp[bodySegment1].SetJointTargetRotation(a[++i], a[++i], 0);
        bp[bodySegment2].SetJointTargetRotation(a[++i], a[++i], 0);

        bp[bodySegment0].SetJointStrength(a[++i]);
        bp[bodySegment1].SetJointStrength(a[++i]);
        bp[bodySegment2].SetJointStrength(a[++i]);

        if (bodySegment0.position.y < startPos.y - 2)
            EndEpisode();
    }

    void FixedUpdate()
    {
    UpdateOrientationObjects();
    // 2. Calcula distancia al depredador
    float dist = Vector3.Distance(bodySegment0.position, predator.position);
    float delta = dist - previousDist;

    // 3. Recompensa por alejarse o castigo por acercarse
    AddReward(delta * 0.02f);

    // 4. Recompensa por sobrevivir este step
    AddReward(0.001f);

    previousDist = dist;
    }

    void UpdateOrientationObjects()
    {
        if (m_DirectionIndicator)
        {
            m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        }
        // dirección de escape = desde predator hacia worm
        Vector3 escapeDir = (bodySegment0.position - predator.position).normalized;

        Vector3 escapePos = bodySegment0.position + escapeDir;
        if (m_TargetTemp == null)
        {
            m_TargetTemp = new GameObject("EscapeTarget").transform;
        }
        m_TargetTemp.position = escapePos;
        m_OrientationCube.UpdateOrientation(bodySegment0, m_TargetTemp);

    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
        for (int i = 0; i < continuousActions.Length; i++)
            continuousActions[i] = 0f; // placeholder
    }
}




