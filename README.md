# GPP_FlowField
<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="Flow Field"></a>

<!-- ABOUT THE PROJECT -->
## About-the-project
![FlowFieldGiphy](https://user-images.githubusercontent.com/104839344/212175190-50d1a982-c059-4a62-834f-fd2484f53796.gif)


In this project I implement a pathfinding algorithm called Flow Field 
that is very efficient for handeling a big group of agents when it comes to pathfinding. 
I created this project For gameplay programming a course i follow at Howest [DAE]

Written in c++, using a framework provided by DAE

<p align="right">(<a href="#readme-top">back to top</a>)</p>



### Built With

* [C++]


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## What are Flow Fields

A Flow Field is also known as vector field pathfinding, this is a technique that calculates the path from the goal to every node in the graph. 
It can be explained in three steps.

*1. First we create a heatmap that calculates the distance between the goal node and every other node on the graph

![HeatMap](https://user-images.githubusercontent.com/104839344/212181669-48c5e57c-56f5-4446-b43f-afaf9c6d7808.JPG)

The idea is that with creating this heatMap we calculate the path distance to the goal for every node so that 
later on (in step 2) we can use these distances to create directionVectors

*2. When we have created our heatmap we can use that information to create a vectorMap

#### To calculate these vectors:
- for every node we create vectors that points towards their neighbours 
- we set the length of each vector to the normalized distance (we calculated this in step 1) of the neigbour node it points to
- we take the average of all these vectors combined and save this direction in our node

this way u wil have a vector that point upwards the gradient (so to the +- to the neighbour node with te lowest distance).
Like mentionedbefore u do this for every node until every node has a vector

![VectorMap](https://user-images.githubusercontent.com/104839344/212182718-876c6758-bf49-4097-8adb-6d7e3f428a86.JPG)

*3. Set the velocity of your agent to the vectors u have created

![agent to goal](https://user-images.githubusercontent.com/104839344/212186247-268b802b-1eb9-455d-8d62-4f17dda19a1d.JPG)

This is the last step and the easiest one, you just have to check which node is at the position of your agent
and then set the agents velocity to that of the vector that is present in that node * your agents maxVelocity

### Design/Implementation 

#### Lets first Talk about our heatMap and how we calculated it:

- first i created an openList and a closedList that represents if a node has already been visited or not (openList = not visited, closedList = visted)
```
std::vector<NodeRecord> openList, closedList;
```
    
- The First element I add is my goalNode, i do this because typically the goal node is added first when creating a heatMap. It is used as the starting point for the search, the algorithm works by expanding out from the goalNode and evaluating the cost of reaching other nodes in the map.

```
openList.push_back(startRecord);
```
- We use a while loop that breaks if the open list is empty meaning al the nodes have been visited
```
while (!openList.empty())
    {
    }
 ```
 
* npm
  ```sh
  npm install npm@latest -g
  ```

### Installation

1. Get a free API Key at [https://example.com](https://example.com)
2. Clone the repo
   ```sh
   git clone https://github.com/github_username/repo_name.git
   ```
3. Install NPM packages
   ```sh
   npm install
   ```
4. Enter your API in `config.js`
   ```js
   const API_KEY = 'ENTER YOUR API';
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources.

_For more examples, please refer to the [Documentation](https://example.com)_

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ROADMAP -->
## Roadmap

- [ ] Feature 1
- [ ] Feature 2
- [ ] Feature 3
    - [ ] Nested Feature

See the [open issues](https://github.com/github_username/repo_name/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Your Name - [@twitter_handle](https://twitter.com/twitter_handle) - email@email_client.com

Project Link: [https://github.com/github_username/repo_name](https://github.com/github_username/repo_name)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* []()
* []()
* []()

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[DAE]: https://www.digitalartsandentertainment.be/
[C++]: https://cplusplus.com/

