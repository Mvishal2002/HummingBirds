using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Flower : MonoBehaviour
{
    public Color FullFlowerColor = new Color(1f, 0f, 0.3f);
    public Color emptyFlowerColor = new Color(0.5f, 0f, 1f);

    [HideInInspector]
    public Collider nectarCollider;

    private Collider flowerCollider;

    private Material flowerMaterial;

    public Vector3 flowerUpVector
    {
        get
        {
            return nectarCollider.transform.up;
        }
    }

    public Vector3 flowerCenterPosition
    {
        get
        {
            return nectarCollider.transform.position;
        }
    }

    public float nectarAmount { get; private set; }

    public bool HasNectar
    {
        get
        {
            return nectarAmount > 0f;
        }
    }

    public float feed(float amount)
    {
        float nectarTaken = Mathf.Clamp(amount, 0f, nectarAmount);

        nectarAmount -= amount;

        if(nectarAmount <= 0)
        {
            nectarAmount = 0;

            flowerCollider.gameObject.SetActive(false);
            nectarCollider.gameObject.SetActive(false);

            flowerMaterial.SetColor("_BaseColor", emptyFlowerColor);
        }

        return nectarTaken;
    }

    public void ResetFlower()
    {
        nectarAmount = 1f;

        flowerCollider.gameObject.SetActive(true);
        nectarCollider.gameObject.SetActive(true);

        flowerMaterial.SetColor("_BaseColor", FullFlowerColor);
    }

    private void Awake()
    {
        MeshRenderer meshRenderer = GetComponent<MeshRenderer>();
        flowerMaterial = meshRenderer.material;

        flowerCollider = transform.Find("FlowerCollider").GetComponent<Collider>();
        nectarCollider = transform.Find("FlowerNectarCollider").GetComponent<Collider>();
    }
}
