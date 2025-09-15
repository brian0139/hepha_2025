package org.firstinspires.ftc.teamcode;
public class Pathfinding {
        public class artifact_DFS {
            //class for artifacts
            public class artifact {
                int x;
                int y;
                String color;
                boolean taken;
            }
            private artifact[][] artifactColorReference = new artifact[4][7];
            private void init(artifact_DFS self) {
                //color reference chart
                for (int i=1;i<=3;i++){
                    for (int j=1;j<=6;j++){
                        //set color
                        if (i==j || (i+j)==7){
                            artifactColorReference[i][j].color="green";
                        }
                        else{
                            artifactColorReference[i][j].color="purple";
                        }
                        //set x value
                        artifactColorReference[i][j].x=28*(i-1)-14;
                        //TO-DO: set y values
                    }
                }
            }
            //TO-DO: write pathing algorithm
            public int[] optimal_path_DFS() {
                int[] x = new int[1];
                return x;
            }
        }
}
