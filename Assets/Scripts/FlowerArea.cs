using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FlowerArea : MonoBehaviour
{
    public const float areaDiameter = 20f;

    private List<GameObject> flowerPlants;

    private Dictionary<Collider, Flower> nectarFlowerDictionary;

    public List<Flower> flowers { get; private set; }

    public void ResetFlowers()
    {
        foreach(GameObject flowerPlant in flowerPlants)
        {
            float xRot = UnityEngine.Random.Range(-5f, 5f);
            float yRot = UnityEngine.Random.Range(-180f, 180f);
            float zRot = UnityEngine.Random.Range(-5f, 5f);
            flowerPlant.transform.localRotation = Quaternion.Euler(xRot, yRot, zRot);
        }

        foreach(Flower flower in flowers)
        {
            flower.ResetFlower();
        }
    }

    public Flower GetFlowerFromNectar(Collider collider)
    {
        return nectarFlowerDictionary[collider];
    }

    private void Awake()
    {
        flowerPlants = new List<GameObject>();
        nectarFlowerDictionary = new Dictionary<Collider, Flower>();
        flowers = new List<Flower>();
        FindChildFlowers(transform);
    }

    private void FindChildFlowers(Transform parent)
    {
        for(int i = 0; i<parent.childCount; i++)
        {
            Transform child = parent.GetChild(i);
            if (child.CompareTag("flower_plant"))
            {
                flowerPlants.Add(child.gameObject);
                FindChildFlowers(child);
            }
            else
            {
                Flower flower = child.GetComponent<Flower>();
                if(flower != null)
                {
                    flowers.Add(flower);
                    nectarFlowerDictionary.Add(flower.nectarCollider, flower);
                }
                else
                {
                    FindChildFlowers(child);
                }
            }
        }
    }
}
