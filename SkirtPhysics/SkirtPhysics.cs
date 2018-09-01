using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class SkirtPhysics : MonoBehaviour {

	[System.Serializable]
	public struct SkirtRing
	{
			[SerializeField] public float radiusX;
			[SerializeField] public float radiusZ;
			[SerializeField] public Vector3 offset;
	}

	public Transform m_root = null;
	public List<Transform> m_meshBones = new List<Transform>();
	[SerializeField]
	public List<SkirtRing> m_rings = new List<SkirtRing>();
	public int m_segments = 8;
	public float m_startingAngle = 22.5f;
	public float m_inertia = 1.0f;
	public float m_damping = 0.1f;
	public float m_shapeStiffness = 0.2f;
	public float m_elasticity = 0.1f;
	public float m_verticalStretchLimit = 1.1f;
	public float m_horizontalStretchLimit = 2.0f;
	public float m_collisionRadius = 0.15f;
	public List<Collider> m_colliders = new List<Collider>();

	class SkirtVertex {
		public Vector3 startLocalPosition;
		public Vector3 position;
		public Vector3 force;
		public Vector3 velocity;
	}
	
	class VertexSpring {
		public bool vertical;
		public bool isV1Static;
		public Vector3 rest;
		public float restLength;
		public SkirtVertex v1;
		public SkirtVertex v2;
	}

	class SkirtBone {
		public Transform boneTransform;
		public Vector3 offset;
		public Quaternion rotationOffset;
		public SkirtVertex[,] anchors;
	}

	SkirtVertex[,] m_verts = null;
	VertexSpring[] m_springs = null;
	GameObject m_dummyObject = null;
	CapsuleCollider m_dummyCapsule = null;
	List<SkirtBone> m_bones = null;

	// Use this for initialization
	void Start () {
		m_verts = GenerateSkirtMesh();
		GenerateSprings();
		GenerateBones();
		m_dummyObject = new GameObject("helper");
		m_dummyObject.transform.parent = transform;
		m_dummyCapsule = m_dummyObject.AddComponent<CapsuleCollider>();
		m_dummyCapsule.direction = 2; // Z axis
		m_dummyObject.SetActive(false);
	}

	void Stop() {
		Destroy(m_dummyObject);
		m_dummyObject = null;
		m_dummyCapsule = null;
	}
	
	// Update is called once per frame
	void Update () {
		
	}

	void OnValidate() {
		m_inertia = Mathf.Clamp01(m_inertia);
		m_damping = Mathf.Clamp01(m_damping);
		m_segments = Mathf.Max(m_segments, 4);
		m_shapeStiffness = Mathf.Max(m_shapeStiffness, 0f);
		m_elasticity = Mathf.Clamp01(m_elasticity);
		m_verticalStretchLimit = Mathf.Max(m_verticalStretchLimit, 1.0f);
		m_horizontalStretchLimit = Mathf.Max(m_horizontalStretchLimit, 1.0f);
		if (m_verts != null) {
			if (m_verts.GetLength(1) != m_segments) {
				m_verts = GenerateSkirtMesh();
			}
		}
	}

	void FixedUpdate() {
		var rootTransform = m_root ?? transform;
		if (m_verts == null || m_springs == null) {
			return;
		}
		for (int s = 0; s < m_segments; ++s) {
			m_verts[0, s].position = rootTransform.TransformPoint(m_verts[0, s].startLocalPosition);
		}

		// Calculating forces
		foreach (var vert in m_verts) {
			var desiredPosition = rootTransform.TransformPoint(vert.startLocalPosition);
			vert.force = (desiredPosition - vert.position) * m_shapeStiffness;
		}

		foreach (var spring in m_springs) {
			var k = m_elasticity * 0.25f;
			var springForce = ((spring.v2.position - spring.v1.position) - spring.rest) * k;
			if (!spring.isV1Static) {
				spring.v1.force += springForce;
			}
			spring.v2.force -= springForce;
		}

		// Moving particles
		foreach (var vert in m_verts) {
			vert.velocity = vert.velocity * (1 - m_damping) + vert.force * m_inertia;
			vert.position += vert.velocity;
		}

		// Handling stretch limits
		foreach (var spring in m_springs) {
			var lenSqr = (spring.v2.position - spring.v1.position).sqrMagnitude;
			var maxLen = spring.restLength * (spring.vertical ? m_verticalStretchLimit : m_horizontalStretchLimit);
			if (lenSqr > maxLen * maxLen) {
				var len = Mathf.Sqrt(lenSqr);
				var shortenDir = (spring.v2.position - spring.v1.position).normalized;
				var v1Delta = shortenDir * (len - maxLen) * 0.5f;
				var v2Delta = -v1Delta;
				if (spring.isV1Static) {
					v1Delta = Vector3.zero;
					v2Delta = -shortenDir * (len - maxLen);
				}
				spring.v1.velocity += v1Delta;
				spring.v1.position += v1Delta;
				spring.v2.velocity += v2Delta;
				spring.v2.position += v2Delta;
			}
		}

		HandleCollisions();
	}

	void HandleCollisions() {
		if (m_dummyCapsule == null || m_dummyObject == null) {
			return;
		}
		var rootTransform = m_root ?? transform;
		m_dummyObject.SetActive(true);
		m_dummyCapsule.radius = m_collisionRadius;
		for (int r = 1; r < m_rings.Count; ++r) {
			for (int s = 0; s < m_segments; ++s) {
				var leftVert = m_verts[r, s];
				var rightVert = m_verts[r, (s + 1) % m_segments];
				m_dummyCapsule.height = (leftVert.position - rightVert.position).magnitude + m_collisionRadius * 2;
				var capsulePos = (leftVert.position + rightVert.position) * 0.5f;
				var forwardDir = leftVert.position - capsulePos;
				var rightDir = (capsulePos - rootTransform.TransformPoint(m_rings[r].offset)).normalized;
				var capsuleRot = Quaternion.LookRotation(forwardDir, Vector3.Cross(rightDir, forwardDir));
				capsulePos = capsulePos + m_collisionRadius * rightDir;
				for (int colliderIdx = 0; colliderIdx < m_colliders.Count; ++colliderIdx) {
					var collider = m_colliders[colliderIdx];
					Vector3 penetrationDir;
					float penetrationDist;
					bool collided = Physics.ComputePenetration(
						m_dummyCapsule, capsulePos, capsuleRot, 
						collider, collider.transform.position, collider.transform.rotation, 
						out penetrationDir, out penetrationDist);
					if (collided) {
						var velocity = penetrationDir * penetrationDist;
						capsulePos = capsulePos + velocity;
						leftVert.position += velocity;
						leftVert.velocity += velocity;
						rightVert.position += velocity;
						rightVert.velocity += velocity;
					}
				}
			}
		}
		m_dummyObject.SetActive(false);
	}

	void GetSegmentOrientation(SkirtVertex[,] corners, out Vector3 position, out Quaternion rotation) {
		position = (corners[0, 0].position + corners[0, 1].position) * 0.5f;
		var direction = (corners[1, 0].position + corners[1, 1].position) * 0.5f - position;
		var normal = Vector3.Cross(corners[0, 1].position - corners[0, 0].position, direction);
		rotation = Quaternion.LookRotation(direction, normal);
	}

	public void LateUpdate() {
		if (m_bones == null) {
			return;
		}
		foreach (var bone in m_bones) {
			Vector3 position;
			Quaternion rotation;
			GetSegmentOrientation(bone.anchors, out position, out rotation);
			bone.boneTransform.SetPositionAndRotation(position + bone.offset, rotation * bone.rotationOffset);
		}
	}

	SkirtVertex[,] GenerateSkirtMesh() {
		var rootTransform = m_root ?? transform;
		var result = new SkirtVertex[m_rings.Count, m_segments];
		for (int ringNum = 0; ringNum < m_rings.Count; ++ringNum) {
			for (int i = 0; i < m_segments; ++i) {
				float angle = Mathf.Deg2Rad * (m_startingAngle + i * 360 / m_segments);
				Vector3 radialOffset = new Vector3(Mathf.Cos(angle) * m_rings[ringNum].radiusX, 0f, Mathf.Sin(angle) * m_rings[ringNum].radiusZ);
				Vector3 localPosition = m_rings[ringNum].offset + radialOffset;
				result[ringNum, i] = new SkirtVertex {
					startLocalPosition = localPosition,
					position = rootTransform.TransformPoint(localPosition)
				};
			}
		}
		return result;
	}

	void GenerateSprings() {
		m_springs = new VertexSpring[(m_rings.Count - 1) * m_segments * 2];
		int springIdx = 0;
		for (int r = 1; r < m_rings.Count; ++r) {
			for (int s = 0; s < m_segments; ++s) {
				var vNeighbor = m_verts[r - 1, s];
				m_springs[springIdx++] = new VertexSpring {
					vertical = true,
					isV1Static = (r == 1),
					rest = (m_verts[r, s].position - vNeighbor.position),
					v1 = vNeighbor,
					v2 = m_verts[r, s]
				};
				var hNeighbor = m_verts[r, (s + 1) % m_segments];
				m_springs[springIdx++] = new VertexSpring {
					vertical = false,
					isV1Static = false,
					rest = (hNeighbor.position - m_verts[r, s].position),
					v1 = m_verts[r, s],
					v2 = hNeighbor
				};
			}
		}
		foreach (var spring in m_springs) {
			spring.restLength = spring.rest.magnitude;
		}
	}

	void GenerateBones() {
		m_bones = new List<SkirtBone>();
		foreach (var meshBone in m_meshBones) {
			int s0 = 0;
			int s1 = 1;
			for (int s = 1; s < m_segments; ++s) {
				int sNext = (s + 1) % m_segments;
				var segmentCenter = (m_verts[0, s].position + m_verts[0, sNext].position) * 0.5f;
				var bestSegmentCenter = (m_verts[0, s0].position + m_verts[0, s1].position) * 0.5f;
				if ((meshBone.position - segmentCenter).sqrMagnitude < (meshBone.position - bestSegmentCenter).sqrMagnitude) {
					s0 = s;
					s1 = sNext;
				}
			}

			var boneTransform = meshBone;
			for (int r = 0; r < m_rings.Count - 1; ++r) {
				m_bones.Add(new SkirtBone {
					boneTransform = boneTransform,
					anchors = new SkirtVertex[2, 2] { {m_verts[r, s0], m_verts[r, s1]}, {m_verts[r + 1, s0], m_verts[r + 1, s1]}}
				});
				if (boneTransform.childCount == 0) {
					break;
				}
				boneTransform = boneTransform.GetChild(0).GetComponent<Transform>();
				if (boneTransform == null) {
					break;
				}
			}
		}

		foreach (var bone in m_bones) {
			Vector3 position;
			Quaternion rotation;
			GetSegmentOrientation(bone.anchors, out position, out rotation);
			bone.offset = bone.boneTransform.position - position;
			bone.rotationOffset = Quaternion.Inverse(rotation) * bone.boneTransform.rotation;
		}
	}

	void OnDrawGizmosSelected() {
		Gizmos.color = new Color(1, 1, 0, 0.75f);
		var verts = m_verts;
		if (verts == null) {
			verts = GenerateSkirtMesh();
		}
		for (int r = 0; r < m_rings.Count; ++r) {
			for (int s = 0; s < m_segments; ++s) {
				Gizmos.DrawLine(verts[r, s].position, verts[r, (s + 1) % m_segments].position);
				if (r < m_rings.Count - 1) {
					Gizmos.DrawLine(verts[r, s].position, verts[r + 1, s].position);
				}
			}
		}
	}
}
