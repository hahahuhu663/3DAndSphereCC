from multiprocessing.pool import ThreadPool
import time

# Define a function that represents a task
def perform_task(task_id):
    time.sleep(0.005) 

start = time.time()
task_ids = range(1, 9)
for task in task_ids:
    print(perform_task(task))
print("time usage (single):", time.time()-start)

start = time.time()
pool = ThreadPool(processes=4)  # You can change the number of threads as needed

    # Define a list of task IDs


    # Use the map method to apply the function to the list of task IDs
results = pool.map(perform_task, task_ids)

    # Close the pool to release resources
pool.close()
pool.join()

    # Print the results
for result in results:
    print(result)

print("time usage (parallel):", time.time()-start)
