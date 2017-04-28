This package provides a versatile ROS knowledge base implementation. The knowledge base uses the tuple space implementation lindypy. 
The facts are modeled as strings.

# Access
The Knowledge Base provides several service, which can be used to read the content of the Knowledge Base or modify its content. For an easier access exists a well documented client (knowledge_base_client.py). The client provides a method for each provided service. All method calls are forwarded to the associated services.

## Patterns
Several services take a pattern instead of a fact. A pattern is similar to a fact, a tuple of strings. However, each position can also contain a placeholder ('*'). A pattern matches all facts, which has the same content at each non-placeholder position.

## Operations
The Knowledge Base provides several services to read and write to the Knowledge Base. The name of each service is the concadenation of the name of the Knowledge Base and the respective postfix. All postfixes are provided as public constants in the class KnowledgeBase. If no name is choosen for the Knowledge Base, the content of the constant KNOWLEDGE_BASE_DEFAULT_NAME is used.
* __Push:__ Adds the given fact to the knowledge base. The fact is not added, if it already stored in the knowledge base
* __Update:__ Takes a pattern, removes all matching facts and add the new fact.
* __Pop:__ Takes a pattern and removes all matching facts. Returns all removed facts.
* __All:__ Takes a pattern ant returns all matching facts
* __Peek:__ Takes a pattern and returns one matching fact.
* __Update Subscribe:__ Takes a pattern and returns three topic names (added, removed, updated). The the Knowledge Base will use these topics, to informs about updates.

## Update Subscribe
The Knowledge Base applies the update subscribe pattern. Therefore returns the UpdateSubscribe-Service the names of three topics. The first one is used, to publishes messages about new facts (message type Fact), the second to inform about removes (message type FactRemoved) and the last about fact replacements (message type FactUpdated). Each topic is only used to inform about facts matching the registered pattern.
It is possible, that the same topics are provided to different client, if they subscribe for the same pattern.
Moreover, a default implementation is provided, which automatically subscribes and handles the update messages. For more details, see to  class KnowledgeBaseFactCache.

# Running
Create an instance of the class KnowledgeBase in a ros node, to use the Knowledge Base. For running it with the default name, a launch file (knowledge_base_node) is provided within this package.