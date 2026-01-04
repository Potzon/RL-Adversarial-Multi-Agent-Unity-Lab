using UnityEngine;
using Unity.MLAgents;

namespace Unity.MLAgentsExamples
{
    [DisallowMultipleComponent]
    public class GroundContact : MonoBehaviour
    {
        [HideInInspector] public Agent agent;

        [Header("Ground Check")] 
        public bool agentDoneOnGroundContact; 
        public bool penalizeGroundContact; 
        public float groundContactPenalty; 

        public float presaContactReward;
        public bool touchingGround;
        const string k_Ground = "ground"; 

        private bool hasTouchedPrey = false;

        void OnCollisionEnter(Collision col)
        {
            // --- Colisión con el suelo ---
            if (col.transform.CompareTag(k_Ground))
            {
                touchingGround = true;
                /* if (penalizeGroundContact)
                    agent.AddReward(groundContactPenalty);
                if (agentDoneOnGroundContact)
                    agent.EndEpisode(); */
            }

            // --- Colisión con la presa ---
            if (col.transform.CompareTag("Prey") && !hasTouchedPrey)
            {
                hasTouchedPrey = true;

                // Asegurarse de que la presa tenga un Agent y que NO sea el propio depredador
                Agent preyAgent = col.transform.GetComponentInParent<Agent>();
                if (preyAgent == null)
                {
                    Debug.LogError("No se encontró Agent en el objeto Prey");
                    return;
                }

                // Evitar falsos positivos por colisiones internas de la presa
                if (preyAgent == agent) return;

                agent.AddReward(presaContactReward);

                // Penalizar y terminar la presa
                preyAgent.AddReward(-5f);

                Debug.Log($"Reward depredador: {agent.GetCumulativeReward()} --- Reward gusano: {preyAgent.GetCumulativeReward()}");
                Debug.Log($"COLLISION: {name} con {col.collider.name} | frame {Time.frameCount}");

                preyAgent.EndEpisode();
                agent.EndEpisode();
            }
        }

        void OnCollisionExit(Collision other)
        {
            if (other.transform.CompareTag(k_Ground))
            {
                touchingGround = false;
                hasTouchedPrey = false;
            }
        }
    }
}
