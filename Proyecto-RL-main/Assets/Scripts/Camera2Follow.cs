using UnityEngine;

namespace Unity.MLAgentsExamples
{
    public class Camera2Follow : MonoBehaviour
    {
        public Transform target1;
        public Transform target2;

        public float smoothingTime = 0.3f;

        [Tooltip("Distancia horizontal desde el punto medio")]
        public float horizontalDistance = 10f;

        [Tooltip("Altura desde el punto medio")]
        public float height = 8f;

        private Vector3 m_CamVelocity;

        void FixedUpdate()
        {
            // Punto medio entre objetivos
            Vector3 midpoint = (target1.position + target2.position) / 2f;

            // Calculamos la posición de la cámara con ángulo
            Vector3 newPosition = midpoint + new Vector3(0, height, -horizontalDistance);

            // Movemos suavemente
            transform.position = Vector3.SmoothDamp(transform.position, newPosition, ref m_CamVelocity, smoothingTime);

            // Miramos hacia el punto medio
            transform.LookAt(midpoint);
        }
    }
}
