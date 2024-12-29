import java.io.FileWriter;
import java.io.IOException;
import java.util.*;
import java.util.concurrent.ThreadLocalRandom;

public class Main {

    private static final int MAX_X = 1000;
    private static final int MAX_Y = 1000;
    private static final double BATERAI_DRONE = 1000; // KAPASITAS BATERAI DRONE
    private static final double PEMAKAIAN_ENERGI = 0.3; // JARAK PER UNIT

    private static int vertexDepot = 0; // default 0

    private double[][] adjacencyMatrix;
    private ArrayList<Vertex> Vertex;
    private List<Integer> bestSolution;
    private double bestSolutionDistance = Double.MAX_VALUE;
    private double bestSolutionEnergy = Double.MAX_VALUE;

    public Main() {
        this.Vertex = new ArrayList<>();
        this.adjacencyMatrix = null;
        this.bestSolution = new ArrayList<>();
    }

    public void randomVertex(int n) {
        Vertex.clear();
        for (int i = 0; i < n; i++) {
            int x = ThreadLocalRandom.current().nextInt(0, MAX_X);
            int y = ThreadLocalRandom.current().nextInt(0, MAX_Y);
            Vertex.add(new Vertex(x, y));
        }
        bestSolution = null;
        hitungAdjacencyMatrix();
    }

    private void hitungAdjacencyMatrix() {
        int numVertex = Vertex.size();
        adjacencyMatrix = new double[numVertex][numVertex];

        for (int i = 0; i < numVertex; i++) {
            for (int j = 0; j < numVertex; j++) {
                if (i == j) {
                    adjacencyMatrix[i][j] = 0;
                } else {
                    adjacencyMatrix[i][j] = HitungJarakManhattan(Vertex.get(i), Vertex.get(j));
                }
            }
        }
    }

    private double HitungJarakManhattan(Vertex v1, Vertex v2) {
        return Math.abs(v1.getX() - v2.getX()) + Math.abs(v1.getY() - v2.getY());
    }

    public void runTSPBfs() {
        if (adjacencyMatrix == null || vertexDepot < 0 || vertexDepot >= adjacencyMatrix.length) {
            throw new IllegalStateException("Adjacency matrix tidak valid.");
        }

        Queue<Path> queue = new LinkedList<>();
        queue.add(new Path(Collections.singletonList(vertexDepot), BATERAI_DRONE));

        while (!queue.isEmpty()) {
            Path currentPath = queue.poll();
            List<Integer> path = currentPath.Vertex;
            double sisaBaterai = currentPath.sisaBaterai;
            int currentVertex = path.get(path.size() - 1);

            if (path.size() == adjacencyMatrix.length) {
                double returnEnergy = adjacencyMatrix[currentVertex][vertexDepot] * PEMAKAIAN_ENERGI;
                if (returnEnergy <= sisaBaterai) {
                    path.add(vertexDepot); // kembali ke depot
                    evaluateSolution(path);
                }
                continue;
            }

            for (int i = 0; i < adjacencyMatrix.length; i++) {
                if (!path.contains(i)) {
                    double requiredEnergy = adjacencyMatrix[currentVertex][i] * PEMAKAIAN_ENERGI;
                    if (requiredEnergy <= sisaBaterai) {
                        List<Integer> newPath = new ArrayList<>(path);
                        newPath.add(i);
                        queue.add(new Path(newPath, sisaBaterai - requiredEnergy));
                    }
                }
            }
        }
    }

    private void evaluateSolution(List<Integer> path) {
        double totalDistance = 0;
        double totalEnergyUsed = 0;

        for (int i = 0; i < path.size() - 1; i++) {
            int current = path.get(i);
            int next = path.get(i + 1);
            double distance = adjacencyMatrix[current][next];
            double energyUsed = distance * PEMAKAIAN_ENERGI;
            totalDistance += distance;
            totalEnergyUsed += energyUsed;
        }

        simpanKeFile(path, totalDistance, totalEnergyUsed);

        if (totalDistance < bestSolutionDistance) {
            bestSolution = new ArrayList<>(path);
            bestSolutionDistance = totalDistance;
            bestSolutionEnergy = totalEnergyUsed;
        }
    }

    private void simpanKeFile(List<Integer> path, double totalDistance, double totalEnergyUsed) {
        try (FileWriter writer = new FileWriter("tsp_solution.txt", true)) {
            writer.write("Path: " + path + "\n");
            writer.write(
                    String.format("Total Jarak: %.2f, Total Energi Digunakan : %.2f\n\n", totalDistance,
                            totalEnergyUsed));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void printBestSolution() {
        if (bestSolution == null || bestSolution.isEmpty()) {
            System.out.println("Tidak Dapat Menemukan Solusi.");
            return;
        }

        System.out.println("Solusi Terbaik TSF BFS : " + bestSolution);
        System.out.printf("Total Jarak: %.2f, Total Energi Digunakan : %.2f%n", bestSolutionDistance,
                bestSolutionEnergy);
        // simpanKeFile(bestSolution, bestSolutionDistance, bestSolutionEnergy);
    }

    public static void main(String[] args) {
        Scanner input = new Scanner(System.in);
        Main tsp = new Main();
        System.out.print("Input Jumlah Vertex : ");
        int numVertex = input.nextInt();
        System.out.print("Input Vertex Depot : ");
        vertexDepot = input.nextInt();
        tsp.randomVertex(numVertex);
        tsp.runTSPBfs();
        tsp.printBestSolution();
        input.close();
    }
}
