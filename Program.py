# -*- coding: utf-8 -*-

import Config


# import cProfile

# cProfile.run('re.compile("foo|bar")')


class Program:
    'Common base class for all programs'
    __drones = []
    __agents = []
    __points = []
    __unnormalized_points = []
    __original_environment = None
    __operator_position = None
    __unnormalized_operator_position = None
    __drone_initial_position = (0, 0)

    def __init__(self):
        return

    # Getters and setters
    def get_drones(self):
        return self.__drones

    def get_points(self):
        return self.__points

    def get_agents(self):
        return self.__agents

    def get_environment(self):
        return self.__original_environment

    def set_environment(self, environment):
        self.__original_environment = environment

    # Other methods
    def normalize_coordinate_value(self, value):
        # Shapely has float point precision errors
        min = -90.0
        max = 90.0

        num = float(value) - min
        denom = max - min

        return num / denom

    def denormalize_coordinate_value(self, norm_value):
        # Shapely has float point precision errors
        min = -90.0
        max = 90.0

        denom = max - min

        return float(norm_value) * denom + min

    def read_data(self):
        # Load data from JSON
        import json
        from Drone import Drone

        with open(Config.BASE_ROUTE + Config.DATA_ROUTE) as json_file:
            data = json.load(json_file)
            self.__drones = []
            for d in data['drones']:  # Drones info
                self.__drones.append(
                    Drone(d['name'].replace(" ", "_"), d['battery_time'], d['speed'],
                          (d['image_size']['w'], d['image_size']['h']),
                          d['height'], d['image_angle']))

            self.__points = []
            for p in data['points']:  # Map info
                self.__points.append(
                    (self.normalize_coordinate_value(p['long']), self.normalize_coordinate_value(p['lat'])))
                self.__unnormalized_points.append((p['long'], p['lat']))
            self.__agents = []

            from shapely.geometry import Point  # Operator's info
            self.__operator_position = Point((self.normalize_coordinate_value(data['operator_position']['long']),
                                              self.normalize_coordinate_value(data['operator_position']['lat'])))
            self.__unnormalized_operator_position = Point((data['operator_position']['long'], data['operator_position']['lat']))

    def compute_minimum_area(self, drones):
        # Get drones minimum image area (supposing a triangle composed of two rectangle triangles)
        areas = []
        for drone in drones:
            '''
            Based on images proyection is composed by Right Scalene Triangle
            Angles:
            A = half image angle
            C = 90
            B = 180 - C - A
            Side:
            a = height
            b = hypotenuse
            c = half base
            '''
            import numpy as np
            a = drone.get_height()
            A = np.deg2rad(drone.get_image_angle() / 2.0)
            C = np.deg2rad(90.0)
            B = np.deg2rad(180.0 - 90 - (drone.get_image_angle() / 2.0))
            b = a * np.sin(C) / np.sin(B)
            c = a * np.sin(A) / np.sin(B)

            image_width = c * 2.0
            image_height = drone.get_image_size()[1] * (image_width / drone.get_image_size()[0])

            areas.append((image_width * image_height, (image_width, image_height)))
        return min(areas, key=lambda t: t[0])[1]

    def compute_random_obstacles(self, obstacle_number, environment):
        from numpy import where
        position_list = where(environment == 1.0)
        valid_position_list = [(i,j) for i,j in zip(position_list[0], position_list[1])]
        from random import shuffle
        shuffle(valid_position_list)
        for pos in valid_position_list[:obstacle_number]:
            environment[pos[0]][pos[1]] = 0.0
        return environment

    def compute_environment(self):
        drones = self.__drones
        points = self.__unnormalized_points
        operator_position = self.__unnormalized_operator_position

        # 1.- Get polygon giving a list of points
        from shapely.geometry import Polygon
        polygon = Polygon(points)
        unnormalized_polygon = Polygon(self.__unnormalized_points)
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        plt.plot(*unnormalized_polygon.exterior.xy)  # Only for Python
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.savefig(Config.BASE_ROUTE + 'field_polygon.png')
        plt.clf()

        # 2.- Get minimum bounding rectangle
        # 2.1.- We need coordinates closest to south (min_x), north (max_x), west (min_y) and east (max_y)
        min_x = min(points, key=lambda t: t[0])[0]
        max_x = max(points, key=lambda t: t[0])[0]
        min_y = min(points, key=lambda t: t[1])[1]
        max_y = max(points, key=lambda t: t[1])[1]

        # 2.2.- Get number of squares verticaly (num_v) and horizontaly (num_h) giving drones' minimum image rectangle
        import math
        if Config.ADJUST_ENVIRONMENT:
            # Get corners of minimum bounding rectangle
            '''
            p1 = (self.denormalize_coordinate_value(max_x), self.denormalize_coordinate_value(min_y))
            p2 = (self.denormalize_coordinate_value(max_x), self.denormalize_coordinate_value(max_y))
            p3 = (self.denormalize_coordinate_value(min_x), self.denormalize_coordinate_value(min_y))
            p4 = (self.denormalize_coordinate_value(min_x), self.denormalize_coordinate_value(max_y))
            '''
            p1 = (max_x,min_y)
            p2 = (max_x,max_y)
            p3 = (min_x,min_y)
            p4 = (min_x,max_y)
            bounding_rectangle = Polygon([p1, p2, p3, p4])

            # Compute vertical distance (d2) and horizontal distance (d1)
            from geopy.distance import geodesic
            d1 = geodesic(p1, p2).meters
            d2 = geodesic(p1, p3).meters

            # Turn minimum bounding rectangle into a numpy matrix of 0s and 1s
            # Get drones' minimum image rectangle
            minimum_image_area = self.compute_minimum_area(drones)

            num_h = math.ceil(d2 / minimum_image_area[0])
            num_v = math.ceil(d1 / minimum_image_area[1])
        else:
            num_v = Config.ENVIRONMENT_ROWS
            num_h = Config.ENVIRONMENT_COLUMNS

        # Maybe we need more squares than default value
        # if (num_v < num_v_drone):
        #     num_v = num_v_drone
        # if (num_h < num_h_drone):
        #     num_h = num_h_drone

        # 3.3.- Create a numpy matrix with a cell for each image square
        import numpy as np
        environment = np.zeros((num_v, num_h))

        # 3.4.- Get coordinates deltas for computing points
        d_v = (max_y - min_y) / num_v
        d_h = (max_x - min_x) / num_h

        # 3.4 Get original operator's point
        if not operator_position.within(polygon):
            from shapely.ops import nearest_points
            closest_point = nearest_points(polygon, operator_position)[0]
        else:
            closest_point = operator_position

        # 3.5.- Check visitable squares as 1
        import itertools
        import random
        for (i, j) in itertools.product(list(range(num_v)), list(range(num_h))):  # i: [0, num_v-1], j: [0, num_h-1]
            sp1 = ((num_h - j) * d_h + min_x, (i) * d_v + min_y)
            sp2 = ((num_h - j) * d_h + min_x, (i + 1) * d_v + min_y)
            sp3 = ((num_h - j + 1) * d_h + min_x, (i + 1) * d_v + min_y)
            sp4 = ((num_h - j + 1) * d_h + min_x, ((i)) * d_v + min_y)
            square = Polygon([sp1, sp2, sp3, sp4])

            if Config.SQUARE:
                environment[num_v - (i + 1), num_h - (j + 1)] = 1.0  # Marked as navigable square

            if polygon.intersects(square.buffer(1e-9)) or polygon.contains(square.buffer(1e-9)):
                # or \
                # square.intersects(polygon.buffer(1e-9)) or square.within(polygon.buffer(1e-9)) or \
                # square.crosses(polygon.buffer(1e-9)) or square.crosses(polygon.buffer(1e-9)):

                if not Config.SQUARE:
                    environment[num_v - (i + 1), num_h - (j + 1)] = 1.0  # Marked as navigable square
                if Config.START_CORNER_0_0 and Config.SQUARE:
                    self.__drone_initial_position = (0, 0)
                elif square.contains(closest_point.buffer(1e-9)) or closest_point.within(square) or closest_point.intersects(square):
                    self.__drone_initial_position = (
                        num_v - (i + 1), num_h - (j + 1))  # Set operator's position as initial position

        environment = self.compute_random_obstacles(Config.NUMBER_OBSTACLES, environment)

        self.__original_environment = environment

        import numpy as np
        np.savetxt(Config.BASE_ROUTE + Config.MAP_ROUTE, environment)

        import matplotlib
        matplotlib.use('Agg')  # For running in SO without graphical environment
        import matplotlib.pyplot as plt
        from matplotlib.ticker import MaxNLocator
        ax = plt.figure().gca()
        ax.invert_yaxis()
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        ax.yaxis.set_major_locator(MaxNLocator(integer=True))
        computed_environment = environment.copy()
        computed_environment[self.__drone_initial_position] = 3
        ax.pcolor(computed_environment, cmap='Greys', edgecolors='gray')
        plt.savefig(Config.BASE_ROUTE + 'computed_environment.png')
        plt.clf()

        return environment

    def reset(self):
        # Reset environment and agents position
        for drone_number in range(len(self.__drones)):
            self.__agents[drone_number].set_position(self.__drone_initial_position)
            self.__agents[drone_number].reset_movements()
            self.__agents[drone_number].set_actions_taken(0)
            self.__agents[drone_number].set_valid_taken_actions(0)

    def save_agents_movements(self, done_count):
        for agent in self.__agents:
            with open(Config.BASE_ROUTE + 'drones_movement_agent' + str(agent.get_number() * 10) + '_done' + str(
                    done_count) + '_.txt',
                      'w') as f:
                f.write(agent.get_name() + ", value %s: " % movement)
                for movement in agent.get_movements:
                    f.write("%s, " % movement)
                f.write('\n')

    def compute_path(self):
        count = 0
        from Agent.Agent import Agent
        self.__agents = []
        for drone in program.get_drones():  # Create Reinforcement Learning Agents
            if not Config.CONTINUE_EXPERIMENT:
                self.__agents.append(
                    Agent(drone.get_name(), count, drone.get_battery_time(),
                          drone.get_speed(),
                          program.compute_minimum_area(self.__drones), (0, 0), self.__original_environment))
            else:
                import pickle
                import os.path
                if os.path.isfile('agent_{0}.pkl'.format(str(count))):
                    with open('agent_' + str(count) + '.pkl', 'rb') as input:
                        self.__agents.append(pickle.load(input))
                else:
                    self.__agents.append(
                        Agent(drone.get_name(), count, drone.get_battery_time(),
                              drone.get_speed(),
                              program.compute_minimum_area(self.__drones), (0, 0), self.__original_environment))
            count += 1

        # Get number of episode episodes
        number_episodes = Config.EPISODES

        import time
        global_execution_start_time = time.time()

        if Config.CONTINUE_EXPERIMENT:
            import json
            with open('checkpoint.json', 'r') as infile:
                data = json.load(infile)
                start_number = data['episode'] + 1
                Config.EPSILON = data['epsilon']
                done_count = data['done_count']  # Number of times problem has been solved
        else:
            start_number = 0
            done_count = 0  # Number of times problem has been solved

        # Get epsilon
        epsilon = Config.EPSILON

        # Save epsilon for plotting
        if Config.CONTINUE_EXPERIMENT:
            epsilons = data['epsilons']
        else:
            epsilons = [epsilon]

        # Total repetitions in all episodes
        total_unchanged_environment_episodes_count = 0

        # Maximum coverage overall
        max_coverage = 0.0

        # Max coverage lists for plotting for the whole experiment
        if Config.CONTINUE_EXPERIMENT:
            max_coverages = data['max_coverages']
        else:
            max_coverages = []

        # Simulations' times
        if Config.CONTINUE_EXPERIMENT:
            episodes_time = data['episodes_time']
        else:
            episodes_time = []

        # Simulations' total rewards
        if Config.CONTINUE_EXPERIMENT:
            rewards_episodes = data['rewards_episodes']
        else:
            rewards_episodes = []

        # Store total actions taken per episode
        if Config.CONTINUE_EXPERIMENT:
            episode_total_actions = data['episode_total_actions']
            episode_total_valid_actions = data['episode_total_valid_actions']
        else:
            episode_total_actions = []
            episode_total_valid_actions = []

        if Config.CONTINUE_EXPERIMENT:
            valid_actions_taken_agent = data['valid_actions_taken_agent']
        else:
            valid_actions_taken_agent = []

        # Compute episodes
        for episode_number in range(start_number, number_episodes):

            # Reset agents and environment
            program.reset()

            # Update heatmap
            heatmap = self.get_environment() * 0.0
            for element in self.__agents:
                (x, y) = element.get_position()
                heatmap[x][y] += 1.0

            # Add minimum max coverage
            max_coverages.append(0.0)

            # Add max coverage episode
            coverages_episode = [0.0]

            # Reset unchanged environments count
            unchanged_environment_episodes_count = 0

            # Create ANN if necessary
            if (Config.GLOBAL_MODEL):
                if not Config.CONTINUE_EXPERIMENT:
                    from numpy import dstack
                    input_matrix = dstack((self.get_environment(), self.get_environment(), self.get_environment()))
                    from Model.Model import create_model
                    model = create_model(input_matrix.shape)
                else:
                    from keras.models import load_model
                    model = load_model('global_model.h5')

            # Get initial environment for starting episode
            actual_environment = program.get_environment()

            # Get visited positions map and agent position map
            import numpy as np
            actual_visited_map = np.array(actual_environment * 0.0, dtype=bool)  # Changed to bool for first experiments
            drone_map = np.array(actual_environment * 0.0, dtype=bool)  # Changed to bool for first experiments

            # Rewards and for plotting
            rewards_episodes.append(0.0)
            rewards = []
            action_rewards = []
            for _ in self.__agents:
                rewards.append([0])
                action_rewards.append([0])

            # Mark agents positions as true solution time
            for agent in self.__agents:
                (i, j) = agent.get_position()
                drone_map[i, j] = True
                actual_visited_map[i, j] = True

            # Print trace every 100 episodes
            if episode_number % Config.SIMULATIONS_CHECKPOINT == 0 and Config.PRINT_EPISODES:
                print("Episode {} of {}".format(episode_number + 1, number_episodes))

            # Compute paths
            done = False
            episode_counter = 0
            visited_list = []  # store each agent's visited squares
            visited_list.append(actual_visited_map)  # store each agent's visited squares

            # Add new values to actions lists
            episode_total_actions.append(0.0)
            episode_total_valid_actions.append(0.0)

            if len(valid_actions_taken_agent):
                for element in self.get_agents():
                    valid_actions_taken_agent[element.get_number()].append(0.0)
            else:
                for _ in self.get_agents():
                    valid_actions_taken_agent.append([0.0])

            # Store trendline_slope
            trendline_slope = -1.0

            import time
            start_time = time.time()
            cicle_count = 1
            while not done:
                cicle_count += 1

                # Get previous environment (this way all agents would act at the same time)
                prev_visited_map = np.array(np.ceil(np.sum(visited_list, axis=0)), dtype=bool).copy()
                prev_drone_map = drone_map.copy()
                drone_position_list = []  # store each agent's position

                # For each agent compute 1 action
                for agent in program.get_agents():

                    # Make decision
                    import numpy as np
                    rand_number = np.random.random()

                    if rand_number < epsilon:
                        random_action = True
                        # Get random action
                        chosen_action = np.random.randint(0, len(Config.ACTIONS_DICT.keys()))
                    else:
                        random_action = False
                        # Decide one action
                        if not Config.GLOBAL_MODEL:
                            chosen_action = np.argmax(agent.predict(np.array(prev_visited_map, dtype=int),
                                                                    np.array(prev_drone_map, dtype=int),
                                                                    self.get_environment(), ))
                        else:
                            chosen_action = np.argmax(agent.predict_global_model(np.array(prev_visited_map, dtype=int),
                                                                                 np.array(prev_drone_map, dtype=int),
                                                                                 self.get_environment(),
                                                                                 model))

                    episode_total_actions[episode_number] += 1.0

                    # Get agent's position before doing action for printing it in a file
                    prev_position = agent.get_position()

                    # Update environment according to action
                    actual_visited_map, actual_drone_map, reward = agent.do_action(chosen_action,
                                                                                   self.__original_environment,
                                                                                   prev_visited_map, prev_drone_map)

                    (r, c) = agent.get_position()
                    heatmap[r][c] += 1.0

                    # Plot heatmap
                    '''
                    import matplotlib
                    matplotlib.use('Agg')  # For running in SO without graphical environment
                    import matplotlib.pyplot as plt
                    plt.plot(rewards[agent.get_number()])
                    fig, ax = plt.subplots()
                    im = ax.imshow(heatmap)
                    for r in range(Config.ENVIRONMENT_ROWS):
                        for c in range(Config.ENVIRONMENT_COLUMNS):
                            text = ax.text(c, r, heatmap[r, c], ha="center", va="center", color="w")
                    fig.tight_layout()
                    plt.savefig('heatmap_episode_' + str(episode_number) + '.png')
                    plt.clf()
                    
                    '''
                    # Plot agent's reward graph
                    '''
                    from numpy import sum
                    rewards[agent.get_number()].append(sum(rewards[agent.get_number()]) + agent.get_reward())
                    action_rewards[agent.get_number()].append(agent.get_reward())
                    rewards_episodes[episode_number] += agent.get_reward()
                    import matplotlib
                    matplotlib.use('Agg')  # For running in SO without graphical environment
                    import matplotlib.pyplot as plt
                    plt.plot(rewards[agent.get_number()])
                    plt.savefig('total_reward_evolution_drone_' + str(agent.get_number()) + '.png')
                    plt.clf()
                    plt.plot(action_rewards[agent.get_number()])
                    plt.savefig('action_reward_evolution_drone_' + str(agent.get_number()) + '.png')
                    plt.clf()
                    '''

                    if (prev_visited_map != actual_visited_map).any():
                        agent.increase_valid_taken_actions()
                        episode_total_valid_actions[episode_number] += 1.0

                    import matplotlib
                    matplotlib.use('Agg')  # For running in SO without graphical environment
                    import matplotlib.pyplot as plt
                    plt.imshow(prev_visited_map, cmap='Greys', interpolation='nearest')
                    plt.savefig(Config.BASE_ROUTE + 'prev_visited_map.png')
                    plt.clf()
                    plt.imshow(actual_visited_map, cmap='Greys', interpolation='nearest')
                    plt.savefig(Config.BASE_ROUTE + 'actual_visited_map.png')
                    plt.clf()
                    if (prev_visited_map == actual_visited_map).all():
                        unchanged_environment_episodes_count += 1  # Store the number of times in a row that the environment does not change
                    else:
                        unchanged_environment_episodes_count = 0

                    # Save taken action in a file
                    with open(
                            Config.BASE_ROUTE + 'actions_' + str(agent.get_number()) + '_' + agent.get_name() + '.csv',
                            'a+') as f:
                        if not episode_counter:
                            agent.set_status('flying')
                            f.write(
                                'action_code, action_name, prev_position, actual_position, valid, visited, random_action, environment_shape, actions_taken, valid_taken_actions, unchanged_episodes\n')
                        f.write(str(chosen_action) + ', ' + Config.ACTIONS_DICT[chosen_action] + ', ' + str(
                            prev_position) + ', ' + str(agent.get_position()) + ', ' + str(
                            prev_position != agent.get_position())
                                + ', ' + str((prev_position != agent.get_position()) and
                                             (prev_visited_map[agent.get_position()[0], agent.get_position()[1]]))
                                + ', ' + str(random_action)
                                + ', ' + str(self.__original_environment.shape) + ', ' + str(agent.get_actions_taken())
                                + ', ' + str(agent.get_valid_taken_actions()) + ', ' + str(
                            unchanged_environment_episodes_count) + '\n')

                    # Memorize new memory episode
                    observation = (
                        prev_visited_map, actual_visited_map, prev_drone_map, actual_drone_map, chosen_action,
                        reward, agent.get_status())
                    agent.memorize(observation)

                    # Save agent results for merging with the remaining agents
                    visited_list.append(actual_visited_map + (1.0 - self.get_environment()))
                    import matplotlib
                    matplotlib.use('Agg')  # For running in SO without graphical environment
                    import matplotlib.pyplot as plt
                    plt.imshow(np.array(np.ceil(np.sum(visited_list, axis=0)), dtype=bool), cmap='Greys',
                               interpolation='nearest')
                    plt.savefig(Config.BASE_ROUTE + 'combined_visited_list.png')
                    plt.clf()

                    drone_position_list.append(actual_drone_map)

                    # Train
                    if not Config.GLOBAL_MODEL:
                        agent_history = agent.learn(self.get_environment())
                        agent.get_model().save(str(agent.get_number()) + '_local_model.h5')
                    else:
                        agent_history = agent.learn_global_model(self.get_environment(), model)
                        model.save('global_model.h5')

                    import pickle
                    # with open('agent_' + str(agent.get_number()) + '.pkl', 'wb') as output:
                    #    pickle.dump(agent, output, pickle.HIGHEST_PROTOCOL)

                    # Check experiment stopping
                    waiting_hours = float(time.time() - start_time) / 60.0 / 60.0  # Convert seconds to hours

                    import numpy as np
                    borders_matrix = 1.0 - np.ceil(self.get_environment())
                    visited_matrix = np.array(np.ceil(np.sum(visited_list, axis=0)), dtype=float)
                    visited_matrix = np.where(visited_matrix >= 1.0, 1.0, visited_matrix)
                    only_visited_cells_matrix = visited_matrix - borders_matrix

                    visited_cells_count = float(np.count_nonzero(only_visited_cells_matrix == 1.0))
                    visitable_cells_count = float(np.count_nonzero(self.get_environment() == 1.0))
                    coverage = visited_cells_count / visitable_cells_count

                    max_coverage = max(coverage, max_coverage)
                    max_coverages[episode_number] = max(coverage, max_coverages[episode_number])
                    coverages_episode.append(coverage)

                    valid_actions_taken_agent[agent.get_number()][episode_number] = agent.get_valid_taken_actions()

                    if unchanged_environment_episodes_count >= Config.MAXIMUM_UNCHANGED_ENVIRONMENT_EPISODES:
                        total_unchanged_environment_episodes_count += unchanged_environment_episodes_count
                        done = True
                        break
                        # Stop experimenting
                        # import sys
                        # sys.exit("Too much episodes without change")
                    elif waiting_hours >= Config.MAXIMUM_WAIT_HOURS and coverage < Config.COMPLETENESS_COVERAGE:
                        total_unchanged_environment_episodes_count += unchanged_environment_episodes_count
                        done = True
                        break
                        # Stop experimenting
                        # import sys
                        # sys.exit("Too slow finding solution ")

                    # Check if agent had finished
                    if False not in np.array(np.ceil(np.sum(visited_list, axis=0)),
                                             dtype=bool):  # TODO UPDATE BATTERY AND TAKE IT INTO ACCOUNT
                        import matplotlib
                        matplotlib.use('Agg')  # For running in SO without graphical environment
                        import matplotlib.pyplot as plt
                        plt.imshow(actual_visited_map, cmap='Greys', interpolation='nearest')
                        plt.savefig(Config.BASE_ROUTE + 'done_environment_' + str(done_count) + '.png')
                        plt.clf()

                        with open(Config.BASE_ROUTE + 'solution_cicles.txt', 'a+') as f:
                            f.write('Cicles: ' + str(done_count)
                                    + str(cicle_count)
                                    + ' episode: ' + str(episode_number)
                                    + ' epsilon: ' + str(epsilon)
                                    + '\n')
                        done_count += 1
                        done = True  # Stop episode when finished
                        break  # Stop processing agents if done

                episode_counter += 1

                # Combine agents results
                drone_map = np.array(np.sum(drone_position_list, axis=0), dtype=bool)

            # Plot coverages for each episode graph
            if len(coverages_episode) > 1:
                import matplotlib
                matplotlib.use('Agg')  # For running in SO without graphical environment
                import matplotlib.pyplot as plt
                ax = plt.figure().gca()
                ax.set_ylim([0.0, 1.0])
                x = list(range(len(coverages_episode)))
                y = coverages_episode
                from numpy import polyfit
                fit = polyfit(x, y, 1)
                yfit = [n * fit[0] for n in x] + fit[1]
                ax.plot(x, y)
                ax.plot(yfit, 'r--')
                plt.xlabel('Actions taken')
                plt.ylabel('Map coverage')
                plt.savefig('coverages_episode_' + str(episode_number) + '.png')
                plt.clf()

            # Store and plot episode's time
            episodes_time.append((time.time() - start_time) / 3600.0)
            import numpy as np
            average_episode_time = np.average(episodes_time)
            import matplotlib
            matplotlib.use('Agg')  # For running in SO without graphical environment
            import matplotlib.pyplot as plt
            ax = plt.figure().gca()
            ax.plot(episodes_time)
            from matplotlib.ticker import MaxNLocator
            ax.xaxis.set_major_locator(MaxNLocator(integer=True))
            plt.ylabel('Hours')
            plt.xlabel('Episode')
            plt.savefig('episode_time_hours.png')
            plt.clf()

            # Plot rewards graph
            import matplotlib
            matplotlib.use('Agg')  # For running in SO without graphical environment
            import matplotlib.pyplot as plt
            ax = plt.figure().gca()
            ax.plot(rewards_episodes)
            from matplotlib.ticker import MaxNLocator
            ax.xaxis.set_major_locator(MaxNLocator(integer=True))
            plt.ylabel('Reward')
            plt.xlabel('Episode')
            plt.savefig('total_rewards.png')
            plt.clf()

            # Plot valid action percentage per episode graph
            if len(episode_total_valid_actions) > 1:
                import matplotlib
                matplotlib.use('Agg')  # For running in SO without graphical environment
                import matplotlib.pyplot as plt
                import numpy as np
                ax = plt.figure().gca()
                division = np.divide(episode_total_valid_actions, episode_total_actions)
                ax.set_ylim([0.0, 1.0])
                x = list(range(len(division)))
                y = division
                from numpy import polyfit
                fit = polyfit(x, y, 1)
                yfit = [n * fit[0] for n in x] + fit[1]
                ax.plot(x, y)
                ax.plot(yfit, 'r--')
                plt.ylabel('Valid taken actions percentages')
                plt.xlabel('Episode')
                plt.savefig('actions_percentages_episodes.png')
                plt.clf()

                import matplotlib
                matplotlib.use('Agg')  # For running in SO without graphical environment
                import matplotlib.pyplot as plt
                import numpy as np
                ax = plt.figure().gca()
                ax.set_ylim([0.0, 1.0])
                for element in self.get_agents():
                    division = np.divide(valid_actions_taken_agent[element.get_number()], episode_total_actions)
                    x = list(range(len(division)))
                    y = division
                    ax.plot(x, y, label='agent ' + str(element.get_number()))
                plt.ylabel('Valid taken actions percentages')
                plt.xlabel('Episode')
                plt.legend(loc="upper right")
                plt.savefig('percentage_work_per_agent.png')
                plt.clf()

            # Plot coverages graph
            if len(max_coverages) > 1:
                import matplotlib
                matplotlib.use('Agg')  # For running in SO without graphical environment
                import matplotlib.pyplot as plt
                ax = plt.figure().gca()
                ax.set_ylim(bottom=0.0)
                x = list(range(len(max_coverages)))
                y = max_coverages
                from scipy.stats import linregress
                trend = linregress(x, y)
                trendline_slope = trend.slope  # or fit[0]
                with open(Config.BASE_ROUTE + 'trend_slope_coverages.txt', 'w+') as f:
                    f.write('trend_slope: ' + str(trendline_slope) + '\n')
                from numpy import polyfit
                fit = polyfit(x, y, 1)
                yfit = [n * fit[0] for n in x] + fit[1]
                ax.plot(x, y)
                ax.plot(yfit, 'r--')
                plt.ylabel('Coverage')
                plt.xlabel('Episode')
                plt.savefig('coverages.png')
                plt.clf()

            # Plot epsilon graph
            import matplotlib
            matplotlib.use('Agg')  # For running in SO without graphical environment
            import matplotlib.pyplot as plt
            ax = plt.figure().gca()
            ax.plot(epsilons)
            from matplotlib.ticker import MaxNLocator
            ax.xaxis.set_major_locator(MaxNLocator(integer=True))
            plt.ylabel('Epsilon')
            plt.xlabel('Episodes')
            plt.savefig('epsilons.png')
            plt.clf()

            # import numpy as np
            # if (average_episode_time + np.sum(episodes_time)) >= Config.MAX_EXPERIMENT_TIME:
            import json
            experiment_status = {'epsilon': epsilon, 'episode': episode_number, 'done_count': done_count,
                                 'seed': Config.SEED, 'epsilons': epsilons, 'max_coverages': max_coverages,
                                 'valid_actions_taken_agent': valid_actions_taken_agent,
                                 'episode_total_actions': episode_total_actions,
                                 'episode_total_valid_actions': episode_total_valid_actions,
                                 'rewards_episodes': rewards_episodes, 'episodes_time': episodes_time,
                                 'coverages_episode': coverages_episode}
            with open('checkpoint.json', 'w') as outfile:
                json.dump(experiment_status, outfile)
            np.savetxt('visited_map.txt', np.array(np.ceil(np.sum(visited_list, axis=0)), dtype=float))

            # Update epsilon
            # The lower the epsilon, less random actions are taken
            epsilon = max(Config.MIN_EPSILON, epsilon * Config.EPSILON_DECAY)
            epsilons.append(epsilon)

            if Config.USE_RANDOM_SEARCH:
                with open('combinations.csv', 'r') as f:
                    # Get list of lines
                    f.seek(0)
                    contents = f.readlines()
                    # Delete the last line
                    del contents[len(contents) - 1]
                    # Insert new last line
                    with open('combinations.csv', 'w') as f:
                        f.writelines(contents + [str(Config.EPSILON) + ', ' + str(Config.EPSILON_DECAY) + ', ' + str(
                            Config.GAMMA) + ', ' + str(
                            Config.MEMORY_SIZE)
                                                 + ', ' + str(Config.BATCH_SIZE) + ', ' + str(
                            Config.EPOCHS) + ', ' + str(Config.NEW_CELL_REWARD)
                                                 + ', ' + str(Config.VISITED_CELL_REWARD) + ', ' + str(
                            Config.NO_CELL_REWARD) + ', ' + str(
                            Config.CONV1_FILTER_NUMBER)
                                                 + ', ' + str(Config.CONV1_SIZE).replace(', ', '_') + ', ' + str(
                            Config.CONV1_STRIDE).replace(
                            ', ', '_') + ', ' + str(Config.CONV1_ACTIVATION)
                                                 + ', ' + str(Config.CONV2_FILTER_NUMBER) + ', ' + str(
                            Config.CONV2_SIZE).replace(', ',
                                                       '_') + ', ' + str(
                            Config.CONV2_STRIDE).replace(', ', '_')
                                                 + ', ' + str(Config.CONV2_ACTIVATION) + ', ' + str(
                            Config.DENSE1_SIZE).replace(', ',
                                                        '_') + ', ' + str(
                            Config.DENSE1_ACTIVATION)
                                                 + ', ' + str(Config.DENSE2_SIZE) + ', ' + str(
                            Config.DENSE2_ACTIVATION) + ', ' + str(
                            Config.OPTIMIZER)
                                                 + ', ' + str(done_count) + ', ' + str(
                            episode_number + 1) + ', ' + str(
                            float(time.time() - global_execution_start_time) / 60.0 / 60.0)
                                                 + ', ' + str(max_coverage) + ', ' + str(trendline_slope) + ', ' + str(
                            Config.SEED) + '\n'])

    def do_random_search(self):
        with open('combinations.csv', 'a+') as f:
            f.write(
                'epsilon, epsilon_decay, gamma, memory_size, batch_size, epochs, new_reward, visited_reward, no_reward,' +
                ' conv1_filter_number, conv1_size, conv1_stride, conv1_activation, conv2_filter_number, conv2_size, conv2_stride, conv2_activation, '
                'dense1_size, dense1_activation, dense2_size, dense2_activation, optimizer, done_count, episodes_number, execution_time, max_coverage, trend_slope, seed\n')
        for i in range(Config.NUMBER_OF_RANDOM_EXPERIMENTS):
            import numpy as np
            import random
            # activation_list = ['relu', 'sigmoid', 'softmax', 'softplus', 'softsign', 'tanh', 'selu', 'exponential', 'elu']
            activation_list = ['relu', 'sigmoid', 'tanh', 'elu']
            # optimizer_list = ['SGD', 'RMSprop', 'Adam', 'Adadelta', 'Adagrad', 'Adamax', 'Nadam']
            optimizer_list = ['SGD', 'RMSprop', 'Adam', 'Adadelta']
            Config.EPSILON = np.random.uniform(0.45, 0.5)  # np.random.uniform(0.25, 0.5)
            Config.EPSILON_DECAY = np.random.uniform(0.9, 0.9999)  # np.random.uniform(0.8, 0.9999)
            Config.GAMMA = np.random.uniform(0.5, 0.9999)
            Config.MEMORY_SIZE = np.random.randint(1, Config.ENVIRONMENT_COLUMNS * Config.ENVIRONMENT_ROWS / 2)
            Config.BATCH_SIZE = np.random.randint(128)
            Config.EPOCHS = np.random.randint(75)
            Config.NEW_CELL_REWARD = np.random.random() * Config.ENVIRONMENT_COLUMNS * Config.ENVIRONMENT_ROWS
            Config.NO_CELL_REWARD = -1.0 * np.random.random() * Config.ENVIRONMENT_COLUMNS * Config.ENVIRONMENT_ROWS
            Config.VISITED_CELL_REWARD = np.random.random() * Config.NO_CELL_REWARD
            Config.CONV1_FILTER_NUMBER = np.random.randint(1, 512)
            size = np.random.randint(1, 9)
            Config.CONV1_SIZE = (size, size)
            size = np.random.randint(1, 5)
            Config.CONV1_STRIDE = (size, size)
            Config.CONV1_ACTIVATION = random.choice(activation_list)
            Config.CONV2_FILTER_NUMBER = np.random.randint(1, 512)
            size = np.random.randint(1, 9)
            Config.CONV2_SIZE = (size, size)
            size = np.random.randint(1, 5)
            Config.CONV2_STRIDE = (size, size)
            Config.CONV2_ACTIVATION = random.choice(activation_list)
            Config.DENSE1_SIZE = np.random.randint(1, 1024)
            Config.DENSE1_ACTIVATION = random.choice(activation_list)
            Config.DENSE2_ACTIVATION = random.choice(activation_list)
            Config.OPTIMIZER = random.choice(optimizer_list)
            with open('combinations.csv', 'a+') as f:
                f.write(str(Config.EPSILON) + ', ' + str(Config.EPSILON_DECAY) + ', ' + str(
                    Config.GAMMA) + ', ' + str(Config.MEMORY_SIZE)
                        + ', ' + str(Config.BATCH_SIZE) + ', ' + str(Config.EPOCHS) + ', ' + str(
                    Config.NEW_CELL_REWARD)
                        + ', ' + str(Config.VISITED_CELL_REWARD) + ', ' + str(
                    Config.NO_CELL_REWARD) + ', ' + str(Config.CONV1_FILTER_NUMBER)
                        + ', ' + str(Config.CONV1_SIZE) + ', ' + str(Config.CONV1_STRIDE) + ', ' + str(
                    Config.CONV1_ACTIVATION)
                        + ', ' + str(Config.CONV2_FILTER_NUMBER) + ', ' + str(
                    Config.CONV2_SIZE) + ', ' + str(Config.CONV2_STRIDE)
                        + ', ' + str(Config.CONV2_ACTIVATION) + ', ' + str(
                    Config.DENSE1_SIZE) + ', ' + str(
                    Config.DENSE1_ACTIVATION)
                        + ', ' + str(Config.DENSE2_SIZE) + ', ' + str(
                    Config.DENSE2_ACTIVATION) + ', ' + str(Config.OPTIMIZER)
                        + ', ' + str(-1) + ', ' + str(-1) + ', ' +
                        str(-1) + ', ' + str(-1) + ',' + str(-1) + ',' + str(Config.SEED) + '\n')
            self.compute_path()


def specify_random_seed():
    import numpy as np

    if Config.SEED == None and not Config.CONTINUE_EXPERIMENT:
        # Get random seed
        Config.SEED = np.random.randint(1, 255)
    elif Config.CONTINUE_EXPERIMENT:
        import json
        with open('checkpoint.json', 'r') as infile:
            data = json.load(infile)
            Config.SEED = data['seed']

            # 1. Set `PYTHONHASHSEED` environment variable at a fixed value
    import os
    os.environ['PYTHONHASHSEED'] = str(Config.SEED)

    # 2. Set `python` built-in pseudo-random generator at a fixed value
    import random
    random.seed(Config.SEED)

    # 3. Set `numpy` pseudo-random generator at a fixed value
    import numpy as np
    np.random.seed(Config.SEED)

    # 4. Set `tensorflow` pseudo-random generator at a fixed value
    import tensorflow as tf
    if tf.__version__ < '2.0.0':
        tf.set_random_seed(Config.SEED)
    else:
        import tensorflow.compat.v1 as tf
        tf.set_random_seed(Config.SEED)

    # 5. Configure a new global `tensorflow` session
    # if tf.__version__ >= '2.0.0':
    #    import tensorflow.compat.v1 as tf
    #    tf.disable_v2_behavior()
    # import tensorflow.python.keras.backend as K
    from tensorflow.python.keras import backend as K
    session_conf = tf.ConfigProto(intra_op_parallelism_threads=1, inter_op_parallelism_threads=1)
    sess = tf.Session(graph=tf.get_default_graph(), config=session_conf)
    K.set_session(sess)

    # 6. Save seed to a file
    with open(Config.BASE_ROUTE + 'session_seed.txt', 'w') as seed_file:
        seed_file.write(str(Config.SEED) + '\n')
        seed_file.close()


def run_tests():
    import unittest
    from Tests.ProgramTests import ProgramTests
    test_classes_to_run = [ProgramTests]
    loader = unittest.TestLoader()

    suites_list = []
    for test_class in test_classes_to_run:
        suite = loader.loadTestsFromTestCase(test_class)
        suites_list.append(suite)

    big_suite = unittest.TestSuite(suites_list)

    runner = unittest.TextTestRunner()
    result = runner.run(big_suite)

    print('Tests run ', result.testsRun)
    print('Error number: ', len(result.errors))
    print('Errors ', result.errors)
    print('Failure number: ', len(result.failures))
    print('Failures ', result.failures)


if __name__ == '__main__':
    if Config.UNIT_TESTS:
        print('\n\n\nRun unit tests')
        run_tests()

    print('\n\n\nSetting random seed')
    specify_random_seed()

    print('\n\n\nInitializing program')
    program = Program()

    print('\n\n\nReading configuration')
    program.read_data()

    print('\n\n\nCompute flying environment')
    if not (Config.CONTINUE_EXPERIMENT or Config.LOAD_MAP_FILE):
        program.compute_environment()
    else:
        import numpy as np

        program.set_environment(np.loadtxt(Config.BASE_ROUTE + Config.MAP_ROUTE))

    print('\n\n\nCompute flying path')
    if not Config.USE_RANDOM_SEARCH:
        program.compute_path()
    else:
        program.do_random_search()
