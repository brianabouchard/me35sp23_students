'''
by Maddie Pero 

In this example we will get data from Airtable.
'''

'''
These statements allow us to make get requests of the Airtable API & parse that information
'''
import requests # you may need to run 'pip install requests' to install this library
import json 


''' This function makes a get request to the airtable API which will tell us how fast to spin the wheels'''

''' Put the URL for your Airtable Base here'''
URL = 'INPUT_URL_HERE'

#'https://api.airtable.com/v0/' + BaseID + '/' + tableName + '?api_key=' + APIKey

r = requests.get(url = URL, params = {})
'''
The get request data comes in as a json package. We will convert this json package to a python dictionary so that it can be parsed
'''
data = r.json()
print(data)


