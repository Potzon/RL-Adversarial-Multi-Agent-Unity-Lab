using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using Random = UnityEngine.Random;

[RequireComponent(typeof(JointDriveController))] // Required to set joint forces
public class CrawlerAgent : Agent
{

    [Header("Walk Speed")]
    [Range(0.1f, m_maxWalkingSpeed)]
    [SerializeField]
    [Tooltip(
        "The speed the agent will try to match.\n\n" +
        "TRAINING:\n" +
        "For VariableSpeed envs, this value will randomize at the start of each training episode.\n" +
        "Otherwise the agent will try to match the speed set here.\n\n" +
        "INFERENCE:\n" +
        "During inference, VariableSpeed agents will modify their behavior based on this value " +
        "whereas the CrawlerDynamic & CrawlerStatic agents will run at the speed specified during training "
    )]
    //The walking speed to try and achieve
    private float m_TargetWalkingSpeed = m_maxWalkingSpeed;

    const float m_maxWalkingSpeed = 15; //The max walking speed

    //The current target walking speed. Clamped because a value of zero will cause NaNs
    public float TargetWalkingSpeed
    {
        get { return m_TargetWalkingSpeed; }
        set { m_TargetWalkingSpeed = Mathf.Clamp(value, .1f, m_maxWalkingSpeed); }
    }

    //The direction an agent will walk during training.
    [Header("Target To Walk Towards")]
    public Transform TargetPrefab; //Target prefab to use in Dynamic envs
    private Transform m_Target; //Target the agent will walk towards during training.

    private float minDistanceToPrey;
    private bool firstStep;

    [Header("Body Parts")][Space(10)] public Transform body;
    public Transform leg0Upper;
    public Transform leg0Lower;
    public Transform leg1Upper;
    public Transform leg1Lower;
    public Transform leg2Upper;
    public Transform leg2Lower;
    public Transform leg3Upper;
    public Transform leg3Lower;

    //This will be used as a stabilized model space reference point for observations
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;

    //The indicator graphic gameobject that points towards the target
    DirectionIndicator m_DirectionIndicator;
    JointDriveController m_JdController;

    [Header("Foot Grounded Visualization")]
    [Space(10)]
    public bool useFootGroundedVisualization;

    public MeshRenderer foot0;
    public MeshRenderer foot1;
    public MeshRenderer foot2;
    public MeshRenderer foot3;
    public Material groundedMaterial;
    public Material unGroundedMaterial;

    public override void Initialize()
    {
        m_Target = TargetPrefab;

        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();
        m_JdController = GetComponent<JointDriveController>();

        //Setup each body part
        m_JdController.SetupBodyPart(body);
        m_JdController.SetupBodyPart(leg0Upper);
        m_JdController.SetupBodyPart(leg0Lower);
        m_JdController.SetupBodyPart(leg1Upper);
        m_JdController.SetupBodyPart(leg1Lower);
        m_JdController.SetupBodyPart(leg2Upper);
        m_JdController.SetupBodyPart(leg2Lower);
        m_JdController.SetupBodyPart(leg3Upper);
        m_JdController.SetupBodyPart(leg3Lower);
    }

    /// <summary>
    /// Loop over body parts and reset them to initial conditions.
    /// </summary>
    public override void OnEpisodeBegin()
    {

        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        

        //Random start rotation to help generalize
        /* body.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0); */

        /* body.position = GetRandomPos(); */

        UpdateOrientationObjects();

        //Set our goal walking speed
        TargetWalkingSpeed = Random.Range(0.1f, m_maxWalkingSpeed);
        minDistanceToPrey = Vector3.Distance(body.position, m_Target.position);
        firstStep = true;
    }

    Vector3 GetRandomPos()
        {
            Vector3 preyBodyPos = TargetPrefab.position;
            Vector3 pos;
            do
            { 
                pos = new Vector3(
                    Random.Range(-15f, 15f),
                    2.006f,
                    Random.Range(-15f, 15f)
                );
             }
            while (Vector3.Distance(pos, preyBodyPos) < 14f); 
            return pos;
        }

    /// <summary>
    /// Add relevant information on each body part to observations.
    /// </summary>
    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        //GROUND CHECK
        sensor.AddObservation(bp.groundContact.touchingGround); // Is this bp touching the ground

        if (bp.rb.transform != body)
        {
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
        }
    }

    /// <summary>
    /// Loop over body parts to add them to observation.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        // Dirección hacia la presa (normalizada y relativa al agente)
        Vector3 dirToPrey = (m_Target.position - body.position).normalized;
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(dirToPrey));

        // Velocidad de la presa (relativa)
        Rigidbody preyRB = m_Target.GetComponent<Rigidbody>();
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(preyRB.linearVelocity));

        // Distancia a la presa
        sensor.AddObservation(Vector3.Distance(body.position, m_Target.position));

        // Posicion
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(m_Target.position));


        RaycastHit hit;
        float maxRaycastDist = 10;
        if (Physics.Raycast(body.position, Vector3.down, out hit, maxRaycastDist))
        {
            sensor.AddObservation(hit.distance / maxRaycastDist);
        }
        else
            sensor.AddObservation(1);

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // The dictionary with all the body parts in it are in the jdController
        var bpDict = m_JdController.bodyPartsDict;

        var continuousActions = actionBuffers.ContinuousActions;
        var i = -1;
        // Pick a new target joint rotation
        bpDict[leg0Upper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leg1Upper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leg2Upper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leg3Upper].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[leg0Lower].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[leg1Lower].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[leg2Lower].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[leg3Lower].SetJointTargetRotation(continuousActions[++i], 0, 0);

        // Update joint strength
        bpDict[leg0Upper].SetJointStrength(continuousActions[++i]);
        bpDict[leg1Upper].SetJointStrength(continuousActions[++i]);
        bpDict[leg2Upper].SetJointStrength(continuousActions[++i]);
        bpDict[leg3Upper].SetJointStrength(continuousActions[++i]);
        bpDict[leg0Lower].SetJointStrength(continuousActions[++i]);
        bpDict[leg1Lower].SetJointStrength(continuousActions[++i]);
        bpDict[leg2Lower].SetJointStrength(continuousActions[++i]);
        bpDict[leg3Lower].SetJointStrength(continuousActions[++i]);

        UpdateOrientationObjects();

        //Esto es visual
        if (useFootGroundedVisualization)
        {
            foot0.material = m_JdController.bodyPartsDict[leg0Lower].groundContact.touchingGround ? groundedMaterial : unGroundedMaterial;
            foot1.material = m_JdController.bodyPartsDict[leg1Lower].groundContact.touchingGround ? groundedMaterial : unGroundedMaterial;
            foot2.material = m_JdController.bodyPartsDict[leg2Lower].groundContact.touchingGround ? groundedMaterial : unGroundedMaterial;
            foot3.material = m_JdController.bodyPartsDict[leg3Lower].groundContact.touchingGround ? groundedMaterial : unGroundedMaterial;
        }

        AddReward(-0.0005f); // penalización suave

        float currentDist = Vector3.Distance(body.position, m_Target.position);
        float delta = minDistanceToPrey - currentDist;

        if (delta > 0)
        { //Si está más cerca
            if(!firstStep){
                AddReward(delta * 0.5f);

            }
            else
            {
                firstStep = false;
            }
            minDistanceToPrey = currentDist;
        }

        

        Vector3 forward = m_OrientationCube.transform.forward;
        Vector3 toPrey = (m_Target.position - body.position).normalized;
        float alignment = Vector3.Dot(forward, toPrey);
        /* AddReward(alignment * 0.01f); //Si mira hacia la presa

        // 1) Velocidad hacia la presa
        float forwardSpeed = Vector3.Dot(GetAvgVelocity(), toPrey);
        AddReward(forwardSpeed * 0.02f);

        // 2) Penalizar quedarse quieto
        if (GetAvgVelocity().magnitude < 0.2f)
            AddReward(-0.01f);

        // 3) Reward por igualar velocidad objetivo
        Vector3 velGoal = toPrey * TargetWalkingSpeed;
        float velReward = GetMatchingVelocityReward(velGoal, GetAvgVelocity());
        AddReward(velReward * 0.05f); */
    }




    /// <summary>
    /// Update OrientationCube and DirectionIndicator
    /// </summary>
    void UpdateOrientationObjects()
    {
        m_OrientationCube.UpdateOrientation(body, m_Target);
        if (m_DirectionIndicator)
        {
            m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        }
    }

    /// <summary>
    ///Returns the average velocity of all of the body parts
    ///Using the velocity of the body only has shown to result in more erratic movement from the limbs
    ///Using the average helps prevent this erratic movement
    /// </summary>
    Vector3 GetAvgVelocity()
    {
        Vector3 velSum = Vector3.zero;
        Vector3 avgVel = Vector3.zero;

        //ALL RBS
        int numOfRb = 0;
        foreach (var item in m_JdController.bodyPartsList)
        {
            numOfRb++;
            velSum += item.rb.linearVelocity;
        }

        avgVel = velSum / numOfRb;
        return avgVel;
    }

    /// <summary>
    /// Normalized value of the difference in actual speed vs goal walking speed.
    /// </summary>
    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        //distance between our actual velocity and goal velocity
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, TargetWalkingSpeed);

        //return the value on a declining sigmoid shaped curve that decays from 1 to 0
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / TargetWalkingSpeed, 2), 2);
    }
 

     public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
        for (int i = 0; i < continuousActions.Length; i++)
            continuousActions[i] = 0f; // placeholder
    }
}