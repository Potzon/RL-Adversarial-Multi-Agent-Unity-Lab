using UnityEngine;

namespace ithappy.Animals_FREE
{
    [RequireComponent(typeof(CreatureMover))]
    public class MovePlayerInput : MonoBehaviour
    {
        [Header("Camera")]
        [SerializeField]
        private PlayerCamera m_Camera;

        private CreatureMover m_Mover;

        // Inputs abstractos
        private Vector2 m_Axis;
        private bool m_IsRun;
        private bool m_IsJump;

        private Vector3 m_Target;
        private Vector2 m_MouseDelta;
        private float m_Scroll;

        private void Awake()
        {
            m_Mover = GetComponent<CreatureMover>();
        }

        private void Update()
        {
            GatherInput();
            SetInput();
        }

        /// <summary>
        /// Recibe la entrada desde cualquier fuente (jugador o agente)
        /// </summary>
        public void GatherInput()
        {
            m_Axis = Vector2.zero;
            m_IsRun = false;
            m_IsJump = false;

            m_Target = (m_Camera == null) ? Vector3.zero : m_Camera.Target;
            m_MouseDelta = Vector2.zero;
            m_Scroll = 0f;
        }

        /// <summary>
        /// Permite que un agente o jugador inyecte los inputs
        /// </summary>
        public void SetAction(Vector2 moveAxis, bool run, bool jump)
        {
            m_Axis = moveAxis;
            m_IsRun = run;
            m_IsJump = jump;
        }

        public void SetInput()
        {
            if (m_Mover != null)
            {
                m_Mover.SetInput(in m_Axis, in m_Target, in m_IsRun, m_IsJump);
            }

            if (m_Camera != null)
            {
                m_Camera.SetInput(in m_MouseDelta, m_Scroll);
            }
        }
    }
}
